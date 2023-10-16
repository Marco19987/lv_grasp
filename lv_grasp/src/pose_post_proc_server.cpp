/** This node is a service server that can be used to post process the estimated pose from a 6D pose estimation algorithm.
 Service:
        request: 
                -> object_type (std_msgs/string)
                -> optimize_pose (bool)
                -> compute_mean (bool)
                -> frame_to_transform (std_msgs/string)             
        respone:
                -> refined pose (geometry_msgs/pose_stamped)

object_type defines the topic from wich the node will read the 6d poses: "/pose_object_type"
if optimize_pose is True, the service server read the depth map from the camera topic and calls the service for the pose refinement, 
instead, it only retrieves the estimated 6D pose and eventually, if required, computes pre-defined mean of poses

*/

#include "rclcpp/rclcpp.hpp"
#include "lv_grasp_interfaces/srv/pose_post_proc_service.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

class PosePostProcServer : public rclcpp::Node
{
public:
  PosePostProcServer() : Node("pose_post_proc_server")
  {


    // Create the pose subscriber
    subscribe_to_pose_topic("/pose_obj");
    // Create the service server
    server_ = this->create_service<lv_grasp_interfaces::srv::PosePostProcService>(
        "pose_post_proc_service",
        std::bind(&PosePostProcServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<lv_grasp_interfaces::srv::PosePostProcService>::SharedPtr server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<>::SharedPtr depth_map_subscriber_;
  
s

  void pose_obj_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Process the received message here
    // ...
  }
  void subscribe_to_pose_topic(const std::string& topic_name)
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
  void handle_service_request(
      const std::shared_ptr<lv_grasp_interfaces::srv::PosePostProcService::Request> request,
      std::shared_ptr<lv_grasp_interfaces::srv::PosePostProcService::Response> response)
  {
    // Verify object type, if changed change the pose topic and recompute the mean 

    // compute mean

    // 

    // Check the boolean value in the request
    if (request->activate)
    {
      // If activate is true, perform processing and populate the response
      response->refined_pose = geometry_msgs::msg::PoseStamped();
      // Populate other fields in the response as needed
      response->scaled_cuboid_dimensions[0] = 1.0;
      response->scaled_cuboid_dimensions[1] = 2.0;
      response->scaled_cuboid_dimensions[2] = 3.0;
      response->scale_obj = 0.5;
  
      //response->success = true;
    }
    else
    {
      
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePostProcServer>());
  rclcpp::shutdown();
  return 0;
}
