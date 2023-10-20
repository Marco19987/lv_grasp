#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/pose_post_proc_service.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pose_post_proc_client_node");
    rclcpp::Client<uclv_grasp_interfaces::srv::PosePostProcService>::SharedPtr client =
        node->create_client<uclv_grasp_interfaces::srv::PosePostProcService>("pose_post_proc_service");

    auto request = std::make_shared<uclv_grasp_interfaces::srv::PosePostProcService::Request>();

    std_msgs::msg::String object_name;
    object_name.data = "santal_ace";
    std_msgs::msg::String frame_to_transform;
    frame_to_transform.data = "";

    request->object_name = object_name;
    request->optimize_pose = true;
    request->compute_mean = true;
    request->frame_to_transform = frame_to_transform;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service completed");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    rclcpp::shutdown();
    return 0;
}