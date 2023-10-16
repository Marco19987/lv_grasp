/** This node is a service server that can be used to post process the estimated pose from a 6D pose estimation algorithm.
 Service:
        request:
                -> object_name (std_msgs/string)
                -> optimize_pose (bool)
                -> compute_mean (bool)
                -> frame_to_transform (std_msgs/string)
        respone:
                -> refined pose (geometry_msgs/pose_stamped)

The parameter object_name defines the topic from wich the node will read the 6d poses: "/pose_object_name"
if optimize_pose is True, the service server read the depth map from the camera topic and calls the service for the pose refinement,
instead, it only retrieves the estimated 6D pose and eventually, if required, computes a pre-defined mean of poses.
The parameter frame_to_transform is used to transform the pose in the specified frame. If the frame is not specified, the pose is not transformed.
*/

#include "rclcpp/rclcpp.hpp"
#include "lv_grasp_interfaces/srv/pose_post_proc_service.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/opencv.hpp"

#define WIDTH 640
#define HEIGHT 480
#define DEPTH_SCALE 0.001
#define SAMPLE_AVERAGE 10

class PosePostProcServer : public rclcpp::Node
{
public:
  PosePostProcServer() : Node("pose_post_proc_server")
  {
    // Initialize the additional attributes
    sample_pose_ = 0;
    sample_depth_ = 0;
    object_name_ = "";
    grasp_pose_msg_ = geometry_msgs::msg::PoseStamped();
    read_depth_ = false;
    read_pose_ = false;
    success_srv_ = true;
    optimizer_ = true;

    // Create the pose subscriber
    subscribe_to_pose_topic("/pose_obj");

    // Create the depth subscriber
    depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_raw", 1, std::bind(&PosePostProcServer::depth_callback, this, std::placeholders::_1));

    // Create the service server
    server_ = this->create_service<lv_grasp_interfaces::srv::PosePostProcService>(
        "pose_post_proc_service",
        std::bind(&PosePostProcServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<lv_grasp_interfaces::srv::PosePostProcService>::SharedPtr server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;

  int sample_pose_;
  int sample_depth_;
  bool read_depth_;
  bool read_pose_;
  bool success_srv_;
  bool optimizer_;
  std::string object_name_;
  geometry_msgs::msg::PoseStamped estimated_pose_msgs_[SAMPLE_AVERAGE];
  sensor_msgs::msg::Image depth_msgs_[SAMPLE_AVERAGE];
  geometry_msgs::msg::PoseStamped grasp_pose_msg_;

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
        topic_name, 1, std::bind(&PosePostProcServer::pose_obj_callback, this, std::placeholders::_1));
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

  void handle_service_request(
      const std::shared_ptr<lv_grasp_interfaces::srv::PosePostProcService::Request> request,
      std::shared_ptr<lv_grasp_interfaces::srv::PosePostProcService::Response> response)
  {
    // Verify object type, if changed change the pose topic and recompute the mean

    // Compute mean

    // Invoke the depth optimization service if required

    // Convert the pose to the required frame if required
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePostProcServer>());
  rclcpp::shutdown();
  return 0;
}
