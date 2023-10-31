/** This node is a service server that can be used to post process the estimated pose from a 6D pose estimation algorithm.
 Service:
        request:
                -> object_name (std_msgs/string)
                -> optimize_pose (bool)
                -> compute_mean (bool)
                -> frame_to_transform (std_msgs/string)
        respone:
                -> estimated pose (geometry_msgs/pose_stamped)
                -> refined pose (geometry_msgs/pose_stamped)
                -> scale_obj (float)

The parameter object_name defines the topic from wich the node will read the 6d poses: "/pose_object_name"
if optimize_pose is True, the service server read the depth map from the camera topic and calls the service for the pose refinement,
instead, it only retrieves the estimated 6D pose and eventually, if required, computes a pre-defined mean of poses.
The parameter frame_to_transform is used to transform the pose in the specified frame. If the frame is not specified, the pose is not transformed.
*/

#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/pose_post_proc_service.hpp"
#include "depth_optimization_interfaces/srv/depth_optimize.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "uclv_utilities/utilities.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <memory>

#define WIDTH 640
#define HEIGHT 480
#define DEPTH_SCALE 0.001
#define SAMPLE_AVERAGE 100

class PosePostProcServer : public rclcpp::Node
{
public:
  PosePostProcServer() : Node("pose_post_proc_server")
  {
    // Initialize the additional attributes
    sample_pose_ = 0;
    sample_depth_ = 0;
    object_name_ = "santal_ace";
    read_depth_ = false;
    read_pose_ = false;
    success_srv_ = true;

    // Define reetrant cb group
    reentrant_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options_cb_group.callback_group = reentrant_cb_group_;

    // Create the pose subscriber
    subscribe_to_pose_topic("/dope/pose_" + object_name_);

    // Create the depth subscriber
    std::string camera_topic = "/camera/aligned_depth_to_color/image_raw";
    depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, rclcpp::SensorDataQoS(), std::bind(&PosePostProcServer::depth_callback, this, std::placeholders::_1), options_cb_group);

    // Create the depth optimization client
    client = this->create_client<depth_optimization_interfaces::srv::DepthOptimize>("/depth_optimize", rmw_qos_profile_services_default, reentrant_cb_group_);

    // Create the service server
    server_ = this->create_service<uclv_grasp_interfaces::srv::PosePostProcService>(
        "pose_post_proc_service",
        std::bind(&PosePostProcServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Pose post processing service server ready!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscription to the topic: " << camera_topic << "\n");
  }

private:
  rclcpp::Service<uclv_grasp_interfaces::srv::PosePostProcService>::SharedPtr server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
  rclcpp::Client<depth_optimization_interfaces::srv::DepthOptimize>::SharedPtr client;
  rclcpp::CallbackGroup::SharedPtr reentrant_cb_group_; // see https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html
  rclcpp::SubscriptionOptions options_cb_group;

  int sample_pose_;
  int sample_depth_;
  bool read_depth_;
  bool read_pose_;
  bool success_srv_;
  std::string object_name_;
  geometry_msgs::msg::PoseStamped estimated_pose_msgs_[SAMPLE_AVERAGE];
  sensor_msgs::msg::Image depth_msgs_[SAMPLE_AVERAGE];
  depth_optimization_interfaces::srv::DepthOptimize::Response::SharedPtr result_depth_optimization_;

  void pose_obj_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Update the sample count and estimated pose messages
    if (sample_pose_ == SAMPLE_AVERAGE)
    {
      read_pose_ = true;
      sample_pose_ = 0;
    }
    if (sample_pose_ < SAMPLE_AVERAGE)
    {
      estimated_pose_msgs_[sample_pose_] = *msg;
      sample_pose_++;
    }
  }
  void subscribe_to_pose_topic(const std::string &topic_name)
  {
    // Reset the subscriber if it already exists
    if (pose_subscriber_)
    {
      pose_subscriber_.reset();
    }

    // Create the subscriber with the new topic name
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name, rclcpp::SensorDataQoS(), std::bind(&PosePostProcServer::pose_obj_callback, this, std::placeholders::_1), options_cb_group);
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscription to the topic: " << topic_name << "\n");
  }
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (sample_depth_ == SAMPLE_AVERAGE)
    {
      read_depth_ = true;
      sample_depth_ = 0;
    }
    if (sample_pose_ > 0 && sample_depth_ < SAMPLE_AVERAGE)
    {

      depth_msgs_[sample_depth_].data = msg->data;
      depth_msgs_[sample_depth_].step = msg->step;
      depth_msgs_[sample_depth_].is_bigendian = msg->is_bigendian;
      depth_msgs_[sample_depth_].height = msg->height;
      depth_msgs_[sample_depth_].encoding = msg->encoding;
      depth_msgs_[sample_depth_].width = msg->width;
      sample_depth_++;
    }
  }
  void get_real_depth(sensor_msgs::msg::Image &msg, cv::Mat &current_depth)
  {
    current_depth = cv::Mat::zeros(cv::Size(WIDTH, HEIGHT), CV_16UC1);
    current_depth = cv_bridge::toCvCopy(msg, msg.encoding)->image;
  }
  void average_pose(geometry_msgs::msg::PoseStamped &estimated_pose_msg)
  {
    for (int i = 0; i < SAMPLE_AVERAGE; i++)
    {
      estimated_pose_msg.pose.position.x = estimated_pose_msg.pose.position.x + estimated_pose_msgs_[i].pose.position.x;
      estimated_pose_msg.pose.position.y = estimated_pose_msg.pose.position.y + estimated_pose_msgs_[i].pose.position.y;
      estimated_pose_msg.pose.position.z = estimated_pose_msg.pose.position.z + estimated_pose_msgs_[i].pose.position.z;

      estimated_pose_msg.pose.orientation.w = estimated_pose_msg.pose.orientation.w + estimated_pose_msgs_[i].pose.orientation.w;
      estimated_pose_msg.pose.orientation.x = estimated_pose_msg.pose.orientation.x + estimated_pose_msgs_[i].pose.orientation.x;
      estimated_pose_msg.pose.orientation.y = estimated_pose_msg.pose.orientation.y + estimated_pose_msgs_[i].pose.orientation.y;
      estimated_pose_msg.pose.orientation.z = estimated_pose_msg.pose.orientation.z + estimated_pose_msgs_[i].pose.orientation.z;
    }
    estimated_pose_msg.pose.position.x = estimated_pose_msg.pose.position.x / SAMPLE_AVERAGE;
    estimated_pose_msg.pose.position.y = estimated_pose_msg.pose.position.y / SAMPLE_AVERAGE;
    estimated_pose_msg.pose.position.z = estimated_pose_msg.pose.position.z / SAMPLE_AVERAGE;

    double w = estimated_pose_msg.pose.orientation.w / SAMPLE_AVERAGE;
    double x = estimated_pose_msg.pose.orientation.x / SAMPLE_AVERAGE;
    double y = estimated_pose_msg.pose.orientation.y / SAMPLE_AVERAGE;
    double z = estimated_pose_msg.pose.orientation.z / SAMPLE_AVERAGE;

    double quat_norm = std::sqrt(std::pow(w, 2) + std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
    estimated_pose_msg.pose.orientation.w = w / quat_norm;
    estimated_pose_msg.pose.orientation.x = x / quat_norm;
    estimated_pose_msg.pose.orientation.y = y / quat_norm;
    estimated_pose_msg.pose.orientation.z = z / quat_norm;

    estimated_pose_msg.header.frame_id = estimated_pose_msgs_[0].header.frame_id;
  }
  void average_depth(float depth_matrix[HEIGHT * WIDTH], cv::Mat depth_cv[SAMPLE_AVERAGE])
  {
    for (int i = 0; i < SAMPLE_AVERAGE; i++)
    {
      get_real_depth(depth_msgs_[i], depth_cv[i]);
    }

    for (int i = 0; i < HEIGHT; i++)
    {
      for (int j = 0; j < WIDTH; j++)
      {
        for (int k = 0; k < SAMPLE_AVERAGE; k++)
        {
          depth_matrix[i * WIDTH + j] = depth_matrix[i * WIDTH + j] + depth_cv[k].at<uint16_t>(i, j) * DEPTH_SCALE;
        }
        depth_matrix[i * WIDTH + j] = depth_matrix[i * WIDTH + j] / SAMPLE_AVERAGE;
      }
    }
  }

  auto transform_pose(const geometry_msgs::msg::PoseStamped &obj_pose, const std::string &frame_to_transform)
  {
    /* Reading transform camera_T_newframe (camera_frame - down, new_frame - up) */
    bool getTransform_ = false;
    geometry_msgs::msg::TransformStamped nf_Transform_c;
    while (!getTransform_ && rclcpp::ok())
      getTransform_ = uclv::getTransform(this->shared_from_this(), frame_to_transform, obj_pose.header.frame_id, nf_Transform_c);

    Eigen::Isometry3d nf_T_c = uclv::geometry_2_eigen(nf_Transform_c.transform);
    // uclv::print_geometry_transform(nf_Transform_c.transform);

    /* Transform camera_T_object (camera_frame - up, object_frame - down) */
    Eigen::Isometry3d c_T_obj = uclv::geometry_2_eigen(obj_pose.pose);

    /* Calculate frame object-base*/
    Eigen::Isometry3d nf_T_obj;
    nf_T_obj = nf_T_c * c_T_obj;

    /* return the transformed pose */
    geometry_msgs::msg::PoseStamped transformed_pose;
    transformed_pose.header.stamp = obj_pose.header.stamp;
    transformed_pose.header.frame_id = frame_to_transform;
    transformed_pose.pose = uclv::eigen_2_geometry(nf_T_obj);

    return transformed_pose;
  }

  auto handle_depth_optimization_response(const rclcpp::Client<depth_optimization_interfaces::srv::DepthOptimize>::SharedFuture result_depth_optimization)
  {

    RCLCPP_INFO(this->get_logger(), "Depth Optimization Service completed!");
    success_srv_ = result_depth_optimization.get()->success;
    if (success_srv_)
    {
      RCLCPP_INFO(this->get_logger(), "Optimization success! \n");
      RCLCPP_INFO(this->get_logger(), "Optimized pose:\n");
      RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(result_depth_optimization.get()->refined_pose));
      RCLCPP_INFO_STREAM(this->get_logger(), "scale_obj: " << result_depth_optimization.get()->scale_obj);
      this->result_depth_optimization_ = result_depth_optimization.get();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for Service completion...");
    }
  }
  void handle_service_request(
      const std::shared_ptr<uclv_grasp_interfaces::srv::PosePostProcService::Request> request,
      std::shared_ptr<uclv_grasp_interfaces::srv::PosePostProcService::Response> response)
  {
    // initialization
    RCLCPP_INFO(this->get_logger(), "Callback get_grasp_pose_service\n");

    // Verify object type, if changed change the pose topic and recompute the mean
    if (object_name_ != request->object_name.data)
    {
      object_name_ = request->object_name.data;
      subscribe_to_pose_topic("/pose_" + object_name_);
      read_pose_ = false;
      sample_pose_ = 0;
      RCLCPP_INFO(this->get_logger(), "Different object type received\n");
      RCLCPP_INFO(this->get_logger(), "Subscription to the topic: /pose_ + %s \n", object_name_.c_str());
    }

    // if requested, compute the mean of the poses and depth maps
    geometry_msgs::msg::PoseStamped estimated_pose;
    float depth_matrix[HEIGHT * WIDTH];

    if (request->compute_mean == true)
    {
      RCLCPP_INFO(this->get_logger(), "Computing the mean of the poses and depth maps...\n");
      rclcpp::Rate rate(10);
      while ((!read_depth_ || !read_pose_) && rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "Waiting for pose and depth messages... (sample_depth=%d, sample_pose=%d)\n", sample_depth_, sample_pose_);
        rate.sleep();
      }
      // compute poses average
      average_pose(estimated_pose);
      RCLCPP_INFO_STREAM(this->get_logger(), "Average pose:\n");
      RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(estimated_pose));

      // compute depth maps average
      cv::Mat depth_cv[SAMPLE_AVERAGE];
      for (int i = 0; i < HEIGHT * WIDTH; i++)
      {
        depth_matrix[i] = 0.0;
      }
      RCLCPP_INFO(this->get_logger(), "\nComputing the mean of the depth maps...\n");
      average_depth(depth_matrix, depth_cv);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "No mean was required\n");
      rclcpp::Rate rate(10);
      while ((sample_pose_ == 0 || sample_depth_ == 0) && rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "Waiting for pose and depth messages... (sample_depth=%d, sample_pose=%d)\n", sample_depth_, sample_pose_);
        rate.sleep();
      }

      estimated_pose = estimated_pose_msgs_[sample_pose_ - 1];
      RCLCPP_INFO(this->get_logger(), "Readed estimated pose:\n");
      RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(estimated_pose));

      cv::Mat depth_cv;
      get_real_depth(depth_msgs_[sample_depth_ - 1], depth_cv);
      for (int i = 0; i < HEIGHT * WIDTH; i++)
      {
        depth_matrix[i] = 0.0;
      }
      for (int i = 0; i < HEIGHT; i++)
      {
        for (int j = 0; j < WIDTH; j++)
        {
          depth_matrix[i * WIDTH + j] = depth_cv.at<uint16_t>(i, j) * DEPTH_SCALE;
        }
      }
    }

    // Invoke the depth optimization service if required
    if (request->optimize_pose == true)
    {
      RCLCPP_INFO(this->get_logger(), "Optimization required: invoking the depth optimization service...\n");

      while (!client->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
      }
      auto request_depth_optimization = std::make_shared<depth_optimization_interfaces::srv::DepthOptimize::Request>();
      request_depth_optimization->estimated_pose = estimated_pose;
      request_depth_optimization->depth_matrix.resize(WIDTH * HEIGHT);
      for (int i = 0; i < WIDTH * HEIGHT; i++)
        request_depth_optimization->depth_matrix[i] = depth_matrix[i];

      auto result_depth_optimization_feature = client->async_send_request(request_depth_optimization);

      std::future_status status = result_depth_optimization_feature.wait_for(std::chrono::seconds(5));
      if (status == std::future_status::ready)
      {
        handle_depth_optimization_response(result_depth_optimization_feature.share());
        // Convert the pose to the required frame if required
        if (request->frame_to_transform.data != "")
        {
          RCLCPP_INFO(this->get_logger(), "Transforming the pose in the required frame...\n");
          estimated_pose = transform_pose(estimated_pose, request->frame_to_transform.data);
          result_depth_optimization_->refined_pose = transform_pose(result_depth_optimization_->refined_pose, request->frame_to_transform.data);
        }
        if (success_srv_)
        {
          RCLCPP_INFO(this->get_logger(), "The pose has been correctly post processed");
          response->estimated_pose = estimated_pose;
          response->refined_pose = result_depth_optimization_->refined_pose;
          response->scale_obj = result_depth_optimization_->scale_obj;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "ERROR: The pose has not been correctly post processed");
          response->estimated_pose = estimated_pose;
        }
      }
      else
        RCLCPP_ERROR(this->get_logger(), "Invoking depth optimization service error");
    }
    else
    {
      if (request->frame_to_transform.data != "")
      {
        RCLCPP_INFO(this->get_logger(), "Transforming the pose in the required frame...\n");
        estimated_pose = transform_pose(estimated_pose, request->frame_to_transform.data);
      }
      RCLCPP_INFO(this->get_logger(), "The pose has been correctly post processed");
      response->estimated_pose = estimated_pose;
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<PosePostProcServer>());
  rclcpp::executors::MultiThreadedExecutor executor;
  auto post_proc_server_node = std::make_shared<PosePostProcServer>();
  executor.add_node(post_proc_server_node);

  RCLCPP_INFO(post_proc_server_node->get_logger(), "Starting post-processing server node, shut down with CTRL-C");
  executor.spin();
  RCLCPP_INFO(post_proc_server_node->get_logger(), "Keyboard interrupt, shutting down.\n");

  rclcpp::shutdown();
  return 0;
}
