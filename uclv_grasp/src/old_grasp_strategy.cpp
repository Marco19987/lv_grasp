#include <math.h> /* pow */
#include "ros/ros.h"
#include <vision_msgs/Detection3DArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <wsg_32_common/Move.h>
#include <std_srvs/Empty.h>
#include <grasp_dope/quintic_traj.h>
#include <tf/transform_listener.h>

#include <grasp_dope/goal_pose_plan_Action.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <geometric_shapes/shape_operations.h>
#include "slipping_control_common/Slipping_Control_Client.h"

bool success_planning_pp = false;
bool success;
double rate = 40; // Hz
vision_msgs::Detection3DArrayConstPtr box_size;
std::string object_name;
double mesh_scale;
std::string mesh_path;
std::vector<double> homing;


bool executeCB(const grasp_dope::goal_pose_plan_GoalConstPtr &goal, actionlib::SimpleActionServer<grasp_dope::goal_pose_plan_Action> *as, ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *move_group_interface, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface, const moveit::core::JointModelGroup *joint_model_group, moveit::core::RobotStatePtr &kinematic_state)
{

    std::vector<geometry_msgs::Pose> pre_grasp_attemp_vector;
    geometry_msgs::Pose pre_grasp_attemp;
    std::vector<geometry_msgs::Pose> place_attempt;

    Eigen::Matrix3d rotation_start;
    Eigen::Matrix3d rotation_alpha;
    Eigen::Matrix3d rotation_theta;

    ros::NodeHandle temp;

    geometry_msgs::PoseStamped temp_pub;
    geometry_msgs::PoseStamped grasp_pose_stamped;

    std::vector<geometry_msgs::Pose> obj_pose_vector;
    geometry_msgs::Pose obj_pose_ = goal->goal_pose_pick.pose;

    double theta = 0.0;   // inclinazione rispetto all'oggetto
    double offset = 0.12; // offset pre-grasp

    if (object_name == "banana")
    {
        std::vector<double> cad_dimensions;
        cad_dimensions.resize(3);
        ros::param::get("/dope/dimensions/" + object_name, cad_dimensions);
        cad_dimensions.at(0) = cad_dimensions.at(0) * 0.01 * scale_obj;
        cad_dimensions.at(1) = cad_dimensions.at(1) * 0.01 * scale_obj;
        cad_dimensions.at(2) = cad_dimensions.at(2) * 0.01 * scale_obj;

        Eigen::Quaterniond orientation_OW(goal->goal_pose_pick.pose.orientation.w, goal->goal_pose_pick.pose.orientation.x, goal->goal_pose_pick.pose.orientation.y, goal->goal_pose_pick.pose.orientation.z);
        Eigen::Vector3d translation_OW(goal->goal_pose_pick.pose.position.x, goal->goal_pose_pick.pose.position.y, goal->goal_pose_pick.pose.position.z);
        Eigen::Matrix3d rotation_OW(orientation_OW);
        Eigen::Vector3d grasp_banana(0, -0.01, 0);
        Eigen::Vector3d tmp = rotation_OW * grasp_banana;
        // obj_pose_.position.x = obj_pose_.position.x + tmp(0);
        // obj_pose_.position.y = obj_pose_.position.y + tmp(1);
        // obj_pose_.position.z = obj_pose_.position.z + tmp(2);

        int inclination_attempt = 4;
        int points_attempt = 3;

        Eigen::Matrix3d rotation_GO;

        if (rotation_OW(2, 0) >= 0)
        {
            rotation_GO << 0, 0, -1,
                0, -1, 0,
                -1, 0, 0;
        }
        else
        {
            rotation_GO << 0, 0, 1,
                0, 1, 0,
                -1, 0, 0;
        }

        rotation_start = rotation_OW * rotation_GO;

        std::vector<double> attempt_position;
        float min_ = cad_dimensions.at(2) / 2 - cad_dimensions.at(2) / 4;
        float max_ = cad_dimensions.at(2) / 2 + cad_dimensions.at(2) / 4;

        for (int i = round(points_attempt / 2); i < points_attempt; i++)
        {
            attempt_position.push_back((min_ + i * (max_ - min_) / points_attempt) - cad_dimensions.at(2) / 2);
        }
        for (int i = 0; i <= round(points_attempt / 2); i++)
        {
            attempt_position.push_back((min_ + i * (max_ - min_) / points_attempt) - cad_dimensions.at(2) / 2);
        }

        for (int i = 0; i < attempt_position.size(); i++)
        {
            ROS_INFO_STREAM(attempt_position.at(i));
            Eigen::Vector3d grasp_banana(0, -0.01, attempt_position.at(i));
            tmp = rotation_OW * grasp_banana;
            obj_pose_.position.x = obj_pose_.position.x + tmp(0);
            obj_pose_.position.y = obj_pose_.position.y + tmp(1);
            obj_pose_.position.z = obj_pose_.position.z + tmp(2);
            obj_pose_.position.z = obj_pose_.position.z + 0.015;

            pre_grasp_attemp.position = obj_pose_.position;
            pre_grasp_attemp.position.z = pre_grasp_attemp.position.z + offset;

            for (int k = 1; k < inclination_attempt; k++)
            {
                theta = M_PI / 3 + (k * M_PI / (4 * inclination_attempt));
                if (rotation_OW(1, 2) < 0)
                {
                    theta = -theta;
                }

                /* Rotazione attorno all'asse y */
                rotation_theta << cos(theta), 0, sin(theta),
                    0, 1, 0,
                    -sin(theta), 0, cos(theta);

                Eigen::Quaterniond q_(rotation_start * rotation_theta);
                pre_grasp_attemp.orientation.w = q_.w();
                pre_grasp_attemp.orientation.x = q_.x();
                pre_grasp_attemp.orientation.y = q_.y();
                pre_grasp_attemp.orientation.z = q_.z();

                place_attempt.push_back(pre_grasp_attemp);

                pre_grasp_attemp_vector.push_back(pre_grasp_attemp);
                obj_pose_vector.push_back(obj_pose_);
            }
        }
    }

    else
    {
        double alpha = 0.0; // rotazione attorno all'oggetto

        rotation_start << 0, -1, 0,
            -1, 0, 0,
            0, 0, -1;
        int rotation_attempt = 6;
        int inclination_attempt = 4;
        for (int i = 2; i < inclination_attempt; i++)
        {
            theta = (i * M_PI / (3 * inclination_attempt));

            /* Rotazione attorno all'asse y */
            rotation_theta << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);

            for (int k = 0; k < rotation_attempt; k++)
            {
                if (k < rotation_attempt / 2)
                    alpha = -(k * M_PI / rotation_attempt); //- M_PI_2;
                else
                    alpha = (k - floor(rotation_attempt / 2)) * M_PI / rotation_attempt + M_PI / rotation_attempt;

                pre_grasp_attemp.position.x = obj_pose_.position.x + offset * sin(theta) * sin(alpha);
                pre_grasp_attemp.position.y = obj_pose_.position.y + offset * sin(theta) * cos(alpha);
                pre_grasp_attemp.position.z = obj_pose_.position.z + offset * cos(theta);

                /* Matrice di rotazione attorno all'asse z */
                rotation_alpha << cos(alpha), -sin(alpha), 0,
                    sin(alpha), cos(alpha), 0,
                    0, 0, 1;

                Eigen::Quaterniond q_(rotation_start * rotation_alpha * rotation_theta);
                pre_grasp_attemp.orientation.w = q_.w();
                pre_grasp_attemp.orientation.x = q_.x();
                pre_grasp_attemp.orientation.y = q_.y();
                pre_grasp_attemp.orientation.z = q_.z();

                if (k == rotation_attempt / 2 - 1)
                {
                    for (int l = 0; l < rotation_attempt; l++)
                    {
                        place_attempt.push_back(pre_grasp_attemp);
                    }
                }

                pre_grasp_attemp_vector.push_back(pre_grasp_attemp);
                obj_pose_vector.push_back(obj_pose_);
            }
        }
    }

