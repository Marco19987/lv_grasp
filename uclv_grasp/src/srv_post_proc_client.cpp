#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/pose_post_proc_service.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher(geometry_msgs::msg::PoseStamped pose)
        : Node("minimal_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("refined_pose", 10);
        this->pose = pose;
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PosePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing the refined pose");
        publisher_->publish(this->pose);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    geometry_msgs::msg::PoseStamped pose;
};

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
    frame_to_transform.data = "camera_infra2_frame";

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

    auto result_depth_optimization_ = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result_depth_optimization_) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service completed");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    auto result_depth_optimization = result_depth_optimization_.get();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depth Optimization Service completed!");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Optimization success! \n");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Estimated pose: \n");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), geometry_msgs::msg::to_yaml(result_depth_optimization.get()->estimated_pose));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Optimized pose:\n");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), geometry_msgs::msg::to_yaml(result_depth_optimization.get()->refined_pose));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "scale_obj: " << result_depth_optimization.get()->scale_obj);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "cad dimension scaled: ");

    for (int i = 0; i < 3; i++)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), result_depth_optimization.get()->scaled_cuboid_dimension.at(i));
    }

    rclcpp::spin(std::make_shared<PosePublisher>(result_depth_optimization.get()->refined_pose));

    rclcpp::shutdown();
    return 0;
}