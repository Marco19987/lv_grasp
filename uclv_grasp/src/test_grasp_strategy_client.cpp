#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/grasp_selection_strategy_srv.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "uclv_utilities/utilities.hpp"

using namespace std::chrono_literals;

class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher(std::vector<geometry_msgs::msg::PoseStamped> pre_grasp_poses)
        : Node("pre_grasp_poses_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pre_grasp_poses", 10);
        publisher_single_poses_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("sorted_poses", 1);
        publisher_ref_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("reference_pose", 10);

        this->pre_grasp_poses.header.stamp = this->now();
        this->pre_grasp_poses.header.frame_id = "map";
        for (std::size_t i = 0; i < pre_grasp_poses.size(); i++)
        {
            this->pre_grasp_poses.poses.push_back(pre_grasp_poses[i].pose);
        }

        // Sort the pre-grasp poses according to the distance from the reference pose
        reference_pose.header.stamp = this->now(); 
        reference_pose.header.frame_id = "map";
        reference_pose.pose.position.x = 0.2;
        reference_pose.pose.position.y = 0.3;
        reference_pose.pose.position.z = 0.30;


        sorted_pre_grasp_poses = uclv::sort_pre_grasp_poses(reference_pose, pre_grasp_poses);

        timer_1 = this->create_wall_timer(
            500ms, std::bind(&PosePublisher::timer_callback, this));
        timer_2 = this->create_wall_timer(
            10s, std::bind(&PosePublisher::single_pose_callback, this));
        timer_3 = this->create_wall_timer(
            1s, std::bind(&PosePublisher::reference_pose_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing the pre-grasp poses");
        publisher_->publish(this->pre_grasp_poses);
    }
    void single_pose_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing sorted pre-grasp poses");
        for (std::size_t i = 0; i < this->sorted_pre_grasp_poses.size(); i++)
        {
            this->sorted_pre_grasp_poses[i].header.stamp = this->now();
            this->sorted_pre_grasp_poses[i].header.frame_id = "map";
            publisher_single_poses_->publish(this->sorted_pre_grasp_poses[i]);
            rclcpp::sleep_for(100ms);
            if(rclcpp::ok())
                continue;
            else
                break;
        }
    }
    void reference_pose_callback()
    {
        publisher_ref_pose_->publish(this->reference_pose);
    }
    // void sort_pre_grasp_poses(geometry_msgs::msg::PoseStamped reference_pose, std::vector<geometry_msgs::msg::PoseStamped> poses)
    // {
    //     std::stable_sort(poses.begin(), poses.end(),
    //               [this, &reference_pose](const geometry_msgs::msg::PoseStamped &pose1, const geometry_msgs::msg::PoseStamped &pose2)
    //               {
    //                   return distance(pose1, reference_pose) < distance(pose2, reference_pose);
    //               });
    //     this->sorted_pre_grasp_poses = poses;
    // }
    // double distance(geometry_msgs::msg::PoseStamped pose_1, geometry_msgs::msg::PoseStamped pose_2)
    // {
    //     return sqrt(pow(pose_1.pose.position.x - pose_2.pose.position.x, 2) +
    //                 pow(pose_1.pose.position.y - pose_2.pose.position.y, 2) +
    //                 pow(pose_1.pose.position.z - pose_2.pose.position.z, 2));
    // }
    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::TimerBase::SharedPtr timer_2;
    rclcpp::TimerBase::SharedPtr timer_3;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_single_poses_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  publisher_ref_pose_;
    geometry_msgs::msg::PoseArray pre_grasp_poses;
    std::vector<geometry_msgs::msg::PoseStamped> sorted_pre_grasp_poses;
    geometry_msgs::msg::PoseStamped reference_pose;
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
