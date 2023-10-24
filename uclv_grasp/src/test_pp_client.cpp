#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/pick_and_place_traj_srv.hpp"
#include "uclv_utilities/color.h"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class DemoNode : public rclcpp::Node
{
public:

    DemoNode() : Node("demo_node")
    {
        // Create the service client to call the planner service
        this->planner_client_ = create_client<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv>("pp_traj_service", rmw_qos_profile_services_default);

        // Wait for the service to be available
        while (!planner_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
        }

        // Call the planner service
        auto planner_request = std::make_shared<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv::Request>();

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = 0.756;
        pose.pose.position.y = -0.069;
        pose.pose.position.z = 0.681;
        pose.pose.orientation.x = 0.490;
        pose.pose.orientation.y = 0.510;
        pose.pose.orientation.z = -0.506;
        pose.pose.orientation.w = -0.495;


        planner_request->target_frame = "push_extension";
        planner_request->pre_grasp_poses.push_back(pose);

        // - Translation: [0.502, -0.069, 0.437]
        // - Rotation: in Quaternion [-0.220, -0.243, 0.669, 0.667]

        // planner_request->pick_pose.pose.position.x = 0.502;
        // planner_request->pick_pose.pose.position.y = -0.069;
        // planner_request->pick_pose.pose.position.z = 0.437;
        // planner_request->pick_pose.pose.orientation.x = -0.220;
        // planner_request->pick_pose.pose.orientation.y = -0.243;
        // planner_request->pick_pose.pose.orientation.z = 0.669;
        // planner_request->pick_pose.pose.orientation.w = 0.667;
    
        pose.pose.position.z = pose.pose.position.z - 0.03;
        planner_request->pick_pose = pose;

        auto planner_response = planner_client_->async_send_request(planner_request);

        // Wait for the service response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), planner_response) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to call planner service");
            return;
        }

        auto planner_response_ = planner_response.get();

        // Get the trajectory from the response
        bool success_ = planner_response_->success;

        if (success_)
        {
            std::cout << BOLDGREEN << "Planning successfully created " << RESET << std::endl;
        }
        else
        {
            std::cout << BOLDRED << "Planning failed!" << RESET << std::endl;
            rclcpp::shutdown();

        }

        // Print the joint positions of the trajectory
        // std::cout << "Planner response: " << std::endl;
        // print_joint_points();
    }

    

private:
    rclcpp::Client<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv>::SharedPtr planner_client_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto demo_node = std::make_shared<DemoNode>();
    rclcpp::spin(demo_node);

    rclcpp::shutdown();

    return 0;
}
