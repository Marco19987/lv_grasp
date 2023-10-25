#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/pick_and_place_traj_srv.hpp"
#include "uclv_utilities/color.h"
#include "uclv_utilities/utilities.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class DemoNode : public rclcpp::Node
{
public:
    bool read_pose = false;
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

        // Wait for the pose of the object

        this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/dope/pose_santal_ace", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                // Process the received pose message here
                this->pick_pose.header = msg->header;
                this->pick_pose.pose = msg->pose;
                this->read_pose = true;
                std::cout << BOLDWHITE << "Received pose of the object" << RESET << std::endl;
            });
    }

    void reset_pub()
    {
        this->pose_sub.reset();
    }

    void get_planning()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(this->pick_pose));
        auto pick_pose_base_link = uclv::transform_pose(this->shared_from_this(), this->pick_pose, "base_link");
        RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(pick_pose_base_link));

        // orientation push_extension [-0.220, -0.243, 0.669, 0.667]
        pick_pose_base_link.pose.orientation.x = -0.220;
        pick_pose_base_link.pose.orientation.y = -0.243;
        pick_pose_base_link.pose.orientation.z = 0.669;
        pick_pose_base_link.pose.orientation.w = 0.667;

        // orientation ee_fingers [0.945, -0.000, 0.328, -0.000]
        // pick_pose_base_link.pose.orientation.x = 0.945;
        // pick_pose_base_link.pose.orientation.y = -0.000;
        // pick_pose_base_link.pose.orientation.z = 0.328;
        // pick_pose_base_link.pose.orientation.w = -0.000;

        geometry_msgs::msg::PoseStamped pre_grasp_pose = pick_pose_base_link;
        pre_grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z + 0.10;

        // std::cout << BOLDWHITE << "pre-grasp pose: " << RESET << std::endl;
        // RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(pre_grasp_pose));

        geometry_msgs::msg::PoseStamped pre_place_pose = pre_grasp_pose;
        pre_place_pose.pose.position.x = pre_place_pose.pose.position.x + 0.1;
        // std::cout << BOLDWHITE << "pre-place pose: " << RESET << std::endl;
        // RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(pre_place_pose));

        geometry_msgs::msg::PoseStamped place_pose = pre_place_pose;
        place_pose.pose.position.z = place_pose.pose.position.z - 0.03;
        // std::cout << BOLDWHITE << "place_pose pose: " << RESET << std::endl;
        // RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(place_pose));

        std::cout << BOLDWHITE << "Creating planning request..." << RESET << std::endl;
        planner_request = std::make_shared<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv_Request>();
        planner_request->target_frame = "push_extension";
        planner_request->pre_grasp_poses.push_back(pre_grasp_pose);
        planner_request->pick_pose = pick_pose_base_link;
        planner_request->pre_place_pose = pre_place_pose;
        planner_request->place_pose = place_pose;

        std::cout << BOLDWHITE << "Calling planner service..." << RESET << std::endl;
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
    }

private:
    rclcpp::Client<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv>::SharedPtr planner_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    geometry_msgs::msg::PoseStamped pick_pose;
    std::shared_ptr<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv_Request> planner_request;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto demo_node = std::make_shared<DemoNode>();
    // rclcpp::spin(demo_node);

    while (!(demo_node->read_pose) && rclcpp::ok())
    {
        std::cout << BOLDWHITE << "Waiting for the pose of the object..." << RESET << std::endl;
        rclcpp::spin_some(demo_node);
        sleep(1);
    }

    demo_node->reset_pub();

    demo_node->get_planning();

    rclcpp::shutdown();

    return 0;
}
