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

class PosePostProcServer : public rclcpp::Node
{
public:
    PosePostProcServer() : Node("pose_post_proc_server")
    {
        // Create the service server
        server_ = this->create_service<uclv_grasp_interfaces::srv::PosePostProcService>(
            "pose_post_proc_service",
            std::bind(&PosePostProcServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Service<uclv_grasp_interfaces::srv::PosePostProcService>::SharedPtr server_;

    void handle_service_request(const std::shared_ptr<uclv_grasp_interfaces::srv::PosePostProcService::Request> request,
                                std::shared_ptr<uclv_grasp_interfaces::srv::PosePostProcService::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request");
        response->success = true;
        response->pre_grasp_poses.resize(1);
        response->pre_grasp_poses[0].pose.position.x = 0.0;
        response->pre_grasp_poses[0].pose.position.y = 0.0;
        response->pre_grasp_poses[0].pose.position.z = 0.0;
        response->pre_grasp_poses[0].pose.orientation.x = 0.0;
        response->pre_grasp_poses[0].pose.orientation.y = 0.0;
        response->pre_grasp_poses[0].pose.orientation.z = 0.0;
        response->pre_grasp_poses[0].pose.orientation.w = 1.0;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePostProcServer>());
    rclcpp::shutdown();
    return 0;
}
