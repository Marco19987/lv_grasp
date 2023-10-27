/** This node is a service server that can be used to compute a set of pre-grasp poses for a given object pose, object type, and, gripper type.
 *   Service:
        request:
                -> object_type (std_msgs/string)
                -> object_pose (geometry_msgs/PoseStamped)
        respone:
                -> pre_grasp_poses (geometry_msgs/PoseStamped[])
                -> success (bool)
 *
*/

#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/grasp_selection_strategy_srv.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "uclv_utilities/utilities.hpp"

#include <eigen3/Eigen/Geometry>

class GraspSelectionStrategyServer : public rclcpp::Node
{
public:
    GraspSelectionStrategyServer() : Node("grasp_selection_strategy_server")
    {
        // Create the service server
        server_ = this->create_service<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv>(
            "grasp_selection_strategy_service",
            std::bind(&GraspSelectionStrategyServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Service<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv>::SharedPtr server_;

    void handle_service_request(const std::shared_ptr<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv::Request> request,
                                std::shared_ptr<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request");
        response->pre_grasp_poses.resize(1);

        if (request->object_type.data == "apple")
        {
            RCLCPP_INFO(this->get_logger(), "Apple object type recognized");
            response->pre_grasp_poses = this->transform_poses(sphere_pre_grasp_poses(), request->object_pose);
            response->success = true;
        }
        else if (request->object_type.data == "santal_ace")
        {
            RCLCPP_INFO(this->get_logger(), "Santal ace object type recognized");
            response->pre_grasp_poses = this->transform_poses(square_pre_grasp_poses(), request->object_pose);
            response->success = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Object type not recognized");
            response->success = false;
        }
    }
    std::vector<geometry_msgs::msg::PoseStamped> sphere_pre_grasp_poses()
    {
        double offset = 0.12;        // offset pre-grasp [cm]
        double theta = 0.0;          // inclination wrt the object
        double alpha = 0.0;          // rotation around the object
        int rotation_attempt = 6;    // number of rotation attempts
        int inclination_attempt = 4; // number of inclination attempts

        std::vector<geometry_msgs::msg::PoseStamped> pre_grasp_poses;

        geometry_msgs::msg::PoseStamped pre_grasp_pose;

        Eigen::Matrix3d rotation_start;
        Eigen::Matrix3d rotation_alpha;
        Eigen::Matrix3d rotation_theta;

        rotation_start << 0, -1, 0,
            -1, 0, 0,
            0, 0, -1;

        //double intial_angle = std::atan2(object_pose.pose.position.y, object_pose.pose.position.x);
 
        // std::cout << "rotation_start: " << rotation_start;
        // std::cout << "intial_angle: " << intial_angle << std::endl;
        // std::cout << "sin(intial_angle): " << std::sin(intial_angle) << std::endl;
        // std::cout << "cos(intial_angle): " << std::cos(intial_angle) << std::endl;

        for (int i = 0; i < inclination_attempt; i++)
        {
            theta = (i * M_PI / (3 * inclination_attempt));
            // theta = (i * 2 * M_PI / (inclination_attempt)); for the complete sphere

            /* Rotation around y axis */
            rotation_theta << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);

            for (int k = 0; k < rotation_attempt; k++)
            {
                if (k < rotation_attempt / 2)
                    alpha = -(k * M_PI / rotation_attempt);
                else
                    alpha = (k - floor(rotation_attempt / 2)) * M_PI / rotation_attempt + M_PI / rotation_attempt;

                //  alpha = -(k * 2 * M_PI / rotation_attempt); for the complete sphere


                pre_grasp_pose.pose.position.x = offset * sin(theta) * sin(alpha);
                pre_grasp_pose.pose.position.y = offset * sin(theta) * cos(alpha);
                pre_grasp_pose.pose.position.z = offset * cos(theta);

                /* Rotation matrix around the z axis */
                rotation_alpha << cos(alpha), -sin(alpha), 0,
                    sin(alpha), cos(alpha), 0,
                    0, 0, 1;

                Eigen::Quaterniond q_(rotation_start *  rotation_alpha * rotation_theta);
                pre_grasp_pose.pose.orientation.w = q_.w();
                pre_grasp_pose.pose.orientation.x = q_.x();
                pre_grasp_pose.pose.orientation.y = q_.y();
                pre_grasp_pose.pose.orientation.z = q_.z();

                pre_grasp_poses.push_back(pre_grasp_pose);
            }
        }
        return pre_grasp_poses;
    }
    std::vector<geometry_msgs::msg::PoseStamped> square_pre_grasp_poses()
    {
        std::vector<geometry_msgs::msg::PoseStamped> pre_grasp_poses(1);
        pre_grasp_poses[0].pose.position.x = 0.2;
        return pre_grasp_poses;
    }
    std::vector<geometry_msgs::msg::PoseStamped> transform_poses(std::vector<geometry_msgs::msg::PoseStamped> pre_grasp_poses, geometry_msgs::msg::PoseStamped object_pose)
    {
        // This method is used to transform the pre-grasp poses of the end effector from the object frame to the base frame
        std::vector<geometry_msgs::msg::PoseStamped> transformed_poses;
        geometry_msgs::msg::PoseStamped transformed_pose;
        for (std::size_t i = 0; i < pre_grasp_poses.size(); i++)
        {
            transformed_pose.header.frame_id = object_pose.header.frame_id;
            transformed_pose.pose = uclv::transform_product(object_pose.pose, pre_grasp_poses[i].pose);
            transformed_poses.push_back(transformed_pose);
        }
        return transformed_poses;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraspSelectionStrategyServer>());
    rclcpp::shutdown();
    return 0;
}
