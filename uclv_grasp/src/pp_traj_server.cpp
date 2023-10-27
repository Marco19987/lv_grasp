/* This node realizes a ROS2 service to plan from the pick pose until the place one*/
#include "rclcpp/rclcpp.hpp"
#include "uclv_moveit_planner_interface/srv/planner_srv.hpp"
#include "uclv_moveit_planner_interface/srv/attach_detach_srv.hpp"
#include "uclv_grasp_interfaces/srv/pick_and_place_traj_srv.hpp"
#include "uclv_utilities/color.h"
#include "uclv_utilities/utilities.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "uclv_moveit_planner_ros2/scene.h"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class PickAndPlaceServer : public rclcpp::Node
{
public:
    PickAndPlaceServer() : Node("pp_traj_service")
    {
        // Define reetrant cb group
        reentrant_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        options_cb_group.callback_group = reentrant_cb_group_;

        // Create the service
        pp_traj_service_ = this->create_service<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv>(
            "pp_traj_service",
            std::bind(&PickAndPlaceServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
        std::cout << BOLDGREEN << "The server 'pp_traj_service' is ready!" << RESET << std::endl;

        // Create the service client to call the planner service
        this->planner_client_ = create_client<uclv_moveit_planner_interface::srv::PlannerSrv>("planner_service", rmw_qos_profile_services_default, reentrant_cb_group_);

        // Create the service client to call the attach/detach service
        this->obj_client_ = create_client<uclv_moveit_planner_interface::srv::AttachDetachSrv>("obj_service", rmw_qos_profile_services_default, reentrant_cb_group_);
    }

private:
    rclcpp::Client<uclv_moveit_planner_interface::srv::PlannerSrv>::SharedPtr planner_client_;
    rclcpp::Client<uclv_moveit_planner_interface::srv::AttachDetachSrv>::SharedPtr obj_client_;
    rclcpp::Service<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv>::SharedPtr pp_traj_service_;
    moveit_msgs::msg::RobotTrajectory traj_home_pre_grasp_;
    moveit_msgs::msg::RobotTrajectory traj_pre_grasp_pick_;
    moveit_msgs::msg::RobotTrajectory traj_pick_post_grasp_;
    moveit_msgs::msg::RobotTrajectory traj_post_grasp_pre_place_;
    moveit_msgs::msg::RobotTrajectory traj_pre_place_place_;
    rclcpp::CallbackGroup::SharedPtr reentrant_cb_group_; // see https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html
    rclcpp::SubscriptionOptions options_cb_group;

    void
    handle_service_request(const std::shared_ptr<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv::Request> request,
                           std::shared_ptr<uclv_grasp_interfaces::srv::PickAndPlaceTrajSrv::Response> response)
    {
        bool success = false;

        std::shared_ptr<uclv_moveit_planner_interface::srv::PlannerSrv_Response> planner_response_;

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
        auto planner_request = std::make_shared<uclv_moveit_planner_interface::srv::PlannerSrv::Request>();
        std::cout << BOLDGREEN << "Creating request for planning server" << RESET << std::endl;

        // Set the request data
        planner_request->target_frame = request->target_frame;
        bool success_i = false;
        bool success_planning_pp = false;
        int num_attempts = int(request->pre_grasp_poses.size());
        int i = 0;

        std::cout << BOLDWHITE << "Creating collision object" << RESET << std::endl;
        uclv::SceneMoveIt scene(this->shared_from_this(), true);
        auto obj = scene.createCollisionObjFromMesh("santal", request->pick_pose, "/home/sfederico/Documents/cad_models/santal_ace/santal_centered.obj", 0.001);
        
        std::cout << BOLDWHITE << "Created object with ID: " << RESET << obj.id << std::endl;
        while (!success_planning_pp && i < num_attempts)
        {
            std::cout << BOLDWHITE << "Attempt " << i + 1 << " of " << num_attempts << RESET << std::endl;
            // Planning from the actual pose of the robot to the i-esim pre-grasp pose of the object in the joint space
            std::cout << BOLDWHITE << "Planning: " << RESET << std::endl;
            std::cout << BOLDMAGENTA << "   1. CURRENT_POSE -> PRE_GRASP: " << RESET << std::flush;
            planner_response_ = planning_service_call_i(planner_request, request->pre_grasp_poses[i], std::vector<double>(), "joint");
            success_i = planner_response_->success;
            if (success_i)
            {
                std::cout << BOLDGREEN << "SUCCESS! " << RESET << std::endl;
                traj_home_pre_grasp_ = planner_response_->traj;

                if (!uclv::askContinue())
                    break;

                // Planning from the i-esim pre-grasp pose of the object to the pick pose in the cartesian space
                std::cout << BOLDMAGENTA << "   2. PRE_GRASP -> PICK: " << RESET << std::flush;
                // Update the pick pose with the orientation of the pre-grasp pose
                request->pick_pose.pose.orientation = request->pre_grasp_poses[i].pose.orientation;
                planner_response_ = planning_service_call_i(planner_request, request->pick_pose, traj_home_pre_grasp_.joint_trajectory.points.back().positions, "cartesian");
                success_i = planner_response_->success;
                if (success_i)
                {
                    std::cout << BOLDGREEN << "SUCCESS! " << RESET << std::endl;
                    traj_pre_grasp_pick_ = planner_response_->traj;

                    // Object attach
                    std::cout << BOLDWHITE << "Attaching object to the end-effector..." << RESET << std::endl;
                    auto obj_attached = obj_service_call(std::make_shared<uclv_moveit_planner_interface::srv::AttachDetachSrv::Request>(), "attach", obj.id);
                    if (!obj_attached)
                    {
                        std::cout << BOLDRED << "Attached FAILED! Continue anyway..." << RESET << std::endl;
                    }

                    if (!uclv::askContinue())
                        break;

                    // Planning from the pick pose to the post-grasp pose in the cartesian space
                    std::cout << BOLDMAGENTA << "   3. PICK -> POST_GRASP: " << RESET << std::flush;
                    planner_response_ = planning_service_call_i(planner_request, request->pre_grasp_poses[i], traj_pre_grasp_pick_.joint_trajectory.points.back().positions, "cartesian");
                    success_i = planner_response_->success;

                    if (success_i)
                    {
                        request->pick_pose.pose.orientation = request->pre_grasp_poses[i].pose.orientation;
                        std::cout << BOLDGREEN << "SUCCESS! " << RESET << std::endl;
                        traj_pick_post_grasp_ = planner_response_->traj;

                        if (!uclv::askContinue())
                            break;

                        // Planning from the post-grasp pose to the pre-place pose in the joint space
                        std::cout << BOLDMAGENTA << "   4. POST_GRASP -> PRE_PLACE: " << RESET << std::flush;
                        planner_response_ = planning_service_call_i(planner_request, request->pre_place_pose, traj_pick_post_grasp_.joint_trajectory.points.back().positions, "joint");
                        success_i = planner_response_->success;

                        if (success_i)
                        {
                            std::cout << BOLDGREEN << "SUCCESS! " << RESET << std::endl;
                            traj_post_grasp_pre_place_ = planner_response_->traj;

                            if (!uclv::askContinue())
                                break;

                            // Planning from the pre-place pose to the place pose in the cartesian space
                            std::cout << BOLDMAGENTA << "   5. PRE_PLACE -> PLACE: " << RESET << std::flush;
                            planner_response_ = planning_service_call_i(planner_request, request->place_pose, traj_post_grasp_pre_place_.joint_trajectory.points.back().positions, "joint");
                            success_i = planner_response_->success;

                            if (success_i)
                            {
                                std::cout << BOLDGREEN << "SUCCESS! " << RESET << std::endl;
                                traj_pre_place_place_ = planner_response_->traj;
                                success_planning_pp = true;
                                success = true;

                                // Detach obj
                                std::cout << BOLDWHITE << "Detaching object from the end-effector..." << RESET << std::endl;
                                auto obj_detached = obj_service_call(std::make_shared<uclv_moveit_planner_interface::srv::AttachDetachSrv::Request>(), "detach", obj.id);
                                if (!obj_detached)
                                {
                                    std::cout << BOLDRED << "Detached FAILED! Continue anyway..." << RESET << std::endl;
                                }
                            }
                            else
                            {
                                std::cout << BOLDRED << "FAILED! " << RESET << std::endl;
                                std::cout << BOLDWHITE << "Trying with the next attempt.." << RESET << std::endl;
                                obj_service_call(std::make_shared<uclv_moveit_planner_interface::srv::AttachDetachSrv::Request>(), "detach", obj.id);

                                i++;
                            }
                        }
                        else
                        {
                            std::cout << BOLDRED << "FAILED! " << RESET << std::endl;
                            std::cout << BOLDWHITE << "Trying with the next attempt.." << RESET << std::endl;

                            obj_service_call(std::make_shared<uclv_moveit_planner_interface::srv::AttachDetachSrv::Request>(), "detach", obj.id);

                            i++;
                        }
                    }
                    else
                    {
                        std::cout << BOLDRED << "FAILED! " << RESET << std::endl;
                        std::cout << BOLDWHITE << "Trying with the next attempt.." << RESET << std::endl;
                        obj_service_call(std::make_shared<uclv_moveit_planner_interface::srv::AttachDetachSrv::Request>(), "detach", obj.id);

                        i++;
                    }
                }
                else
                {
                    std::cout << BOLDRED << "FAILED! " << RESET << std::endl;
                    std::cout << BOLDWHITE << "Trying with the next attempt.." << RESET << std::endl;
                    obj_service_call(std::make_shared<uclv_moveit_planner_interface::srv::AttachDetachSrv::Request>(), "detach", obj.id);

                    i++;
                }
            }
            else
            {
                std::cout << BOLDRED << "FAILED! " << RESET << std::endl;
                std::cout << BOLDWHITE << "Trying with the next attempt.." << RESET << std::endl;
                i++;
            }

            // Set the response status
            response->success = success;
            if (success)
            {
                std::cout << BOLDGREEN << "Planning for the pick and place successfully created!" << RESET << std::endl;
                response->traj_home_pre_grasp = traj_home_pre_grasp_;
                response->traj_pre_grasp_pick = traj_pre_grasp_pick_;
                response->traj_pick_post_grasp = traj_pick_post_grasp_;
                response->traj_post_grasp_pre_place = traj_post_grasp_pre_place_;
                response->traj_pre_place_place = traj_pre_place_place_;
            }
            else
                std::cout << BOLDRED << "Planning failed!" << RESET << std::endl;
        }
    }

    std::shared_ptr<uclv_moveit_planner_interface::srv::PlannerSrv_Response> planning_service_call_i(std::shared_ptr<uclv_moveit_planner_interface::srv::PlannerSrv_Request> planner_request,
                                                                                                     const geometry_msgs::msg::PoseStamped &pose_, const std::vector<double> &start_joints, std::string planning_type)
    {
        // std::cout << BOLDGREEN << "Creating request for planning server callback" << RESET << std::endl;
        // Planning from the actual pose of the robot to the i-esim pre-grasp pose of the object in the joint space
        ;
        planner_request->start_joints = start_joints;
        planner_request->planning_type = planning_type;
        planner_request->pose = pose_;
        auto planner_response = planner_client_->async_send_request(planner_request);
        // std::cout << BOLDWHITE << "Planning type: " << planning_type << RESET << std::endl;
        // Wait for the service response
        std::future_status status = planner_response.wait_for(std::chrono::seconds(10));
        if (status == std::future_status::ready)
            return (planner_response.get());
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invoking pp_traj_service error");
            return (NULL);
        }
    }

    bool obj_service_call(std::shared_ptr<uclv_moveit_planner_interface::srv::AttachDetachSrv_Request> planner_request, const std::string &type, const std::string &id, const std::string &link_ = "ee_fingers")
    {
        planner_request->type = type;
        planner_request->link_name_to_attach = link_;
        planner_request->obj_id = id;

        auto obj_response = obj_client_->async_send_request(planner_request);

        std::future_status status = obj_response.wait_for(std::chrono::seconds(10));
        if (status == std::future_status::ready)
            return obj_response.get()->success;
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invoking obj_service error");
            return false;
        }
    }

    void print_joint_points(moveit_msgs::msg::RobotTrajectory traj)
    {
        for (size_t i = 0; i < traj.joint_trajectory.points.size(); ++i)
        {
            auto joint_positions = traj.joint_trajectory.points[i].positions;
            std::cout << "Joint positions at point " << i << ": ";
            for (size_t j = 0; j < joint_positions.size(); ++j)
            {
                std::cout << joint_positions[j] << " ";
            }
            std::cout << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto pp_traj_server_node = std::make_shared<PickAndPlaceServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pp_traj_server_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
