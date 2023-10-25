#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/pose_post_proc_service.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

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

    // {
    //     // Set the mesh for the DepthOptimizerServer node
    //     set_parameters_client_ = create_client<rcl_interfaces::srv::SetParameters>("/depth_optimizer_server/set_parameters");

    //     // Wait for service
    //     while (!set_parameters_client_->wait_for_service(std::chrono::seconds(1)))
    //     {
    //         RCLCPP_INFO(node->get_logger(), "Service /depth_optimizer_server/set_parameters non available. Waiting...");
    //     }
    
    //     // Set the parameters
    //     std::vector<rcl_interfaces::msg::Parameter> parameters;
    //     rcl_interfaces::msg::Parameter mesh_path_param;
    //     mesh_path_param.name = "mesh_path";
    //     mesh_path_param.value = "cad.obj";
    //     parameters.push_back(mesh_path_param);
    //     rcl_interfaces::msg::Parameter mesh_scale_param;
    //     mesh_scale_param.name = "mesh_scale";
    //     mesh_scale_param.value = 0.001;
    //     parameters.push_back(mesh_scale_param);

    //     // Send the request
    //     auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    //     request->parameters = parameters;
    //     auto future = set_parameters_client_->async_send_request(request);
    //     rclcpp::spin_until_future_complete(node, future);

    //     if (rclcpp::ok() && future.get()->successful)
    //     {
    //         RCLCPP_INFO(node->get_logger(), "Parameters set");
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to set parameters");
    //     }
    // }

    // Get the 6D pose of the object using the PostProcService node
    auto request = std::make_shared<uclv_grasp_interfaces::srv::PosePostProcService::Request>();

    std_msgs::msg::String object_name;
    object_name.data = "santal_ace";
    std_msgs::msg::String frame_to_transform;
    frame_to_transform.data = "base_link";

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

    // Get the pose and publish it
    auto result_depth_optimization = result_depth_optimization_.get();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depth Optimization Service completed!");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Optimization success! \n");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Estimated pose: \n");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), geometry_msgs::msg::to_yaml(result_depth_optimization.get()->estimated_pose));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Optimized pose:\n");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), geometry_msgs::msg::to_yaml(result_depth_optimization.get()->refined_pose));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "scale_obj: " << result_depth_optimization.get()->scale_obj);

    rclcpp::spin(std::make_shared<PosePublisher>(result_depth_optimization.get()->refined_pose));

    rclcpp::shutdown();
    return 0;
}