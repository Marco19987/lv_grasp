#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/pick_and_place_traj_srv.hpp"
#include "uclv_utilities/color.h"
#include "uclv_utilities/utilities.hpp"
#include "uclv_moveit_planner_interface/action/traj_action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using TrajAction_ = uclv_moveit_planner_interface::action::TrajAction;
using GoalHandleTrajAction = rclcpp_action::ClientGoalHandle<TrajAction_>;

class DemoNode : public rclcpp::Node
{
public:
    bool read_pose = false;
    DemoNode() : Node("demo_node")
    {

        // Create the service client to call the planner service
        this->planner_client_ = create_client<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv>("pp_traj_service", rmw_qos_profile_services_default);

        // Create the action client to call the trajectory execution action
        this->traj_action_client_ = rclcpp_action::create_client<TrajAction_>(this, "trajectory_execution_as");

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

    bool get_planning()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(this->pick_pose));
        auto obj_pose_base_link = uclv::transform_pose(this->shared_from_this(), this->pick_pose, "base_link");
        RCLCPP_INFO_STREAM(this->get_logger(), geometry_msgs::msg::to_yaml(obj_pose_base_link));

        geometry_msgs::msg::TransformStamped transform;
        geometry_msgs::msg::PoseStamped pick_pose_base_link = obj_pose_base_link;

        if (uclv::getTransform(this->shared_from_this(), "base_link", "ee_fingers", transform))
            pick_pose_base_link.pose.orientation = transform.transform.rotation; // 0.985;
        else
        {
            std::cout << BOLDRED << "Failed to get transform" << RESET << std::endl;
            rclcpp::shutdown();
        }

        // pick_pose_base_link.pose.orientation.y = 0.014;
        // pick_pose_base_link.pose.orientation.z = 0.167;
        // pick_pose_base_link.pose.orientation.w = 0.044;

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
        planner_request->target_frame = "ee_fingers";
        planner_request->pre_grasp_poses.push_back(pre_grasp_pose);
        planner_request->pick_pose = obj_pose_base_link;
        planner_request->pre_place_pose = pre_place_pose;
        planner_request->place_pose = place_pose;

        std::cout << BOLDWHITE << "Calling planner service..." << RESET << std::endl;
        auto planner_response = planner_client_->async_send_request(planner_request);

        // Wait for the service response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), planner_response) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to call planner service");
            return false;
        }

        auto planner_response_ = planner_response.get();

        // Get the trajectory from the response
        bool success_ = planner_response_->success;

        traj_vec.clear();
        traj_vec.push_back(planner_response_->traj_home_pre_grasp);
        traj_vec.push_back(planner_response_->traj_pre_grasp_pick);
        traj_vec.push_back(planner_response_->traj_pick_post_grasp);
        traj_vec.push_back(planner_response_->traj_post_grasp_pre_place);
        traj_vec.push_back(planner_response_->traj_pre_place_place);

        if (success_)
        {
            std::cout << BOLDGREEN << "Planning successfully created " << RESET << std::endl;
        }
        else
        {
            std::cout << BOLDRED << "Planning failed!" << RESET << std::endl;
            rclcpp::shutdown();
        }

        return success_;
    }

    void send_goal()
    {
        using namespace std::placeholders;

        std::cout << BOLDWHITE << "Waiting for action server to start." << RESET << std::endl;

        if (!this->traj_action_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = TrajAction_::Goal();
        goal_msg.simulation = false;
        goal_msg.traj = traj_vec;
        goal_msg.topic_robot = "joint_states";
        goal_msg.scale_factor = 1.0;
        goal_msg.rate = 50.0;
        std::cout << BOLDWHITE << "Sending goal to the action server" << RESET << std::endl;

        auto send_goal_options = rclcpp_action::Client<TrajAction_>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&DemoNode::goal_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&DemoNode::result_callback, this, _1);
        this->traj_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    auto getTimer()
    {
        return this->timer_;
    }

private:
    std::vector<moveit_msgs::msg::RobotTrajectory> traj_vec;
    rclcpp::Client<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv>::SharedPtr planner_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    geometry_msgs::msg::PoseStamped pick_pose;
    std::shared_ptr<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv_Request> planner_request;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<TrajAction_>::SharedPtr traj_action_client_;

    void goal_response_callback(const GoalHandleTrajAction::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandleTrajAction::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        if (result.result->success)
            std::cout << BOLDGREEN << "Trajectory execution success!" << RESET << std::endl;
        else
            std::cout << BOLDRED << "Trajectory execution failed!" << RESET << std::endl;

        rclcpp::shutdown();
    }
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

    if (demo_node->get_planning())
    {
        demo_node->send_goal();
        // demo_node->getTimer() = demo_node->create_wall_timer(
        //     std::chrono::milliseconds(500),
        //     std::bind(&DemoNode::send_goal, demo_node));
    };
    
    

    // rclcpp::shutdown();

    return 0;
}
