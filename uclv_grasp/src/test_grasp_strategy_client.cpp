#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/grasp_selection_strategy_srv.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using namespace std::chrono_literals;

class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher(std::vector<geometry_msgs::msg::PoseStamped> pre_grasp_poses)
        : Node("pre_grasp_poses_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pre_grasp_poses", 10);
        
        this->pre_grasp_poses.header.stamp = this->now();
        this->pre_grasp_poses.header.frame_id = "map";
        for(std::size_t i=0; i<pre_grasp_poses.size(); i++)
        {
            this->pre_grasp_poses.poses.push_back(pre_grasp_poses[i].pose);
        }

        timer_ = this->create_wall_timer(
            500ms, std::bind(&PosePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing the pre-grasp poses pose");
        publisher_->publish(this->pre_grasp_poses);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    geometry_msgs::msg::PoseArray pre_grasp_poses;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grasp_selection_strategy_client");

    rclcpp::Client<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv>::SharedPtr client =
        node->create_client<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv>("grasp_selection_strategy_service");

    auto request = std::make_shared<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv::Request>();
    request->object_type.data = "apple";
    request->object_pose.pose.position.x = 0.5;
    request->object_pose.pose.position.y = 0.5;
    request->object_pose.pose.position.z = 0.0;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result_ = client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result_) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service completed");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    // Get the pose and publish it
    auto pre_grasp_poses_ = result_.get();

    rclcpp::spin(std::make_shared<PosePublisher>(pre_grasp_poses_->pre_grasp_poses));

    rclcpp::shutdown();
    return 0;
}
