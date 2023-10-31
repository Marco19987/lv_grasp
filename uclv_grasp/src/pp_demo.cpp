#include "rclcpp/rclcpp.hpp"
#include "uclv_grasp_interfaces/srv/pose_post_proc_service.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "uclv_grasp_interfaces/srv/grasp_selection_strategy_srv.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "uclv_utilities/utilities.hpp"

#include <thread>
using namespace std::chrono_literals;

void set_depth_optimizer_params(std::shared_ptr<rclcpp::Node> node, std::string mesh_path, double mesh_scale)
{
    // Set the mesh for the DepthOptimizerServer node
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_ =
        node->create_client<rcl_interfaces::srv::SetParameters>("/depth_optimizer_server/set_parameters");

    // Wait for service
    while (!set_parameters_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(node->get_logger(), "Service /depth_optimizer_server/set_parameters non available. Waiting...");
    }

    // Set the parameters
    std::vector<rcl_interfaces::msg::Parameter> parameters;

    rcl_interfaces::msg::Parameter mesh_path_param;
    mesh_path_param.name = "mesh_path";
    mesh_path_param.value.type = rclcpp::PARAMETER_STRING;
    mesh_path_param.value.string_value = mesh_path;
    parameters.push_back(mesh_path_param);

    rcl_interfaces::msg::Parameter mesh_scale_param;
    mesh_scale_param.name = "mesh_scale";
    mesh_scale_param.value.type = rclcpp::PARAMETER_DOUBLE;
    mesh_scale_param.value.double_value = mesh_scale;
    parameters.push_back(mesh_scale_param);

    // Send the request
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters = parameters;
    auto future = set_parameters_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(node, future);

    if (rclcpp::ok() && future.get()->results.back().successful)
    {
        RCLCPP_INFO(node->get_logger(), "Parameters set");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to set parameters");
    }
}
uclv_grasp_interfaces::srv::PosePostProcService::Response::SharedPtr invoke_pose_post_proc_server(std::shared_ptr<rclcpp::Node> node, std_msgs::msg::String object_name, bool optimize_pose, bool compute_mean, std_msgs::msg::String frame_to_transform)
{
    rclcpp::Client<uclv_grasp_interfaces::srv::PosePostProcService>::SharedPtr client =
        node->create_client<uclv_grasp_interfaces::srv::PosePostProcService>("pose_post_proc_service");

    // Get the 6D pose of the object using the PostProcService node
    auto request = std::make_shared<uclv_grasp_interfaces::srv::PosePostProcService::Request>();

    request->object_name = object_name;
    request->optimize_pose = optimize_pose;
    request->compute_mean = compute_mean;
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
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service post_proc_service");
    }

    // return the result
    auto result_depth_optimization = result_depth_optimization_.get();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depth Optimization Service completed!");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Optimization success! \n");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Estimated pose: \n");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), geometry_msgs::msg::to_yaml(result_depth_optimization.get()->estimated_pose));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Optimized pose:\n");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), geometry_msgs::msg::to_yaml(result_depth_optimization.get()->refined_pose));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "scale_obj: " << result_depth_optimization.get()->scale_obj);

    return result_depth_optimization;
}
uclv_grasp_interfaces::srv::GraspSelectionStrategySrv::Response::SharedPtr invoke_grasp_selection_strategy_server(
                std::shared_ptr<rclcpp::Node> node, std::string object_type, 
                geometry_msgs::msg::PoseStamped object_pose)
{
        rclcpp::Client<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv>::SharedPtr client =
        node->create_client<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv>("grasp_selection_strategy_service");

    auto request = std::make_shared<uclv_grasp_interfaces::srv::GraspSelectionStrategySrv::Request>();
    request->object_type.data = object_type;
    request->object_pose = object_pose;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
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
    return result_.get();
}

class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher()
        : Node("pose_publisher")
    {
    }
    int publish_pose(std::string topicname, double queue_size, geometry_msgs::msg::PoseStamped pose)
    {
        pose_publishers++;
        publisher_poses[pose_publishers] = this->create_publisher<geometry_msgs::msg::PoseStamped>(topicname, queue_size);
        this->poses[pose_publishers] = pose;
        timer_poses[pose_publishers] = this->create_wall_timer(
            1s, [this]()
            { pose_callback(this->pose_publishers); });

        return pose_publishers;
    }
    int publish_pose_array(std::string topicname, double queue_size, geometry_msgs::msg::PoseArray pose_array)
    {
        pose_arrays_publishers++;
        publisher_pose_arrays[pose_arrays_publishers] = this->create_publisher<geometry_msgs::msg::PoseArray>(topicname, queue_size);
        this->pose_arrays[pose_arrays_publishers] = pose_array;
        timer_pose_arrays[pose_arrays_publishers] = this->create_wall_timer(
            1s, [this]()
            { pose_array_callback(this->pose_arrays_publishers); });

        return pose_arrays_publishers;
    }

private:
    void pose_callback(int index)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing the refined pose");
        publisher_poses[index]->publish(this->poses[index]);
    }
    void pose_array_callback(int index)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing array pose");
        publisher_pose_arrays[index]->publish(this->pose_arrays[index]);
    }

    std::vector<rclcpp::TimerBase::SharedPtr> timer_poses;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publisher_poses;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    int pose_publishers = -1;

    std::vector<rclcpp::TimerBase::SharedPtr> timer_pose_arrays;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> publisher_pose_arrays;
    std::vector<geometry_msgs::msg::PoseArray> pose_arrays;
    int pose_arrays_publishers = -1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pp_demo_node");
    rclcpp::executors::MultiThreadedExecutor executor;
    auto pose_publisher = std::make_shared<PosePublisher>();
    executor.add_node(pose_publisher);

    std::thread spin_thread([&executor]()
                            { executor.spin(); });

    // ################################### USER INPUT ###############################################

    // ------------- depth_optimizer_server params -------------
    std::string mesh_path = "/home/sfederico/Documents/cad_models/Apple/Apple_4K/food_apple_01_4k.obj";
    double mesh_scale = 1.0;

    // ------------- pose_post_proc_service params -------------
    std_msgs::msg::String object_name;
    std_msgs::msg::String frame_to_transform;
    object_name.data = "santal_ace";
    frame_to_transform.data = "camera_link";
    bool optimize_pose = false;
    bool compute_mean = true;
    
    // ------------- grasp_selection_strategy_service params -------------
    std::string object_type = "apple";
    std::string base_frame = "base_link";
    std::string ee_frame = "ee_link";

    // ################################## RETRIEVE OBJECT POSE ######################################
    set_depth_optimizer_params(node, mesh_path, mesh_scale);

    auto result_depth_optimization = invoke_pose_post_proc_server(node, object_name, optimize_pose, compute_mean, frame_to_transform);

    pose_publisher->publish_pose("refined_pose", 10, result_depth_optimization->refined_pose);
    pose_publisher->publish_pose("estimated_pose", 10, result_depth_optimization->estimated_pose);

    // ################################## RETRIEVE PRE-GRASP POSES ######################################
    auto pre_grasp_poses_ = invoke_grasp_selection_strategy_server(node, object_type, result_depth_optimization->refined_pose);

    // sort the pre-grasp poses according to the distance from the end effector pose 
    geometry_msgs::msg::TransformStamped end_effector_pose_transform;  
    uclv::getTransform(node, base_frame, ee_frame ,end_effector_pose_transform);
    auto sorted_pre_grasp_poses = uclv::sort_pre_grasp_poses(uclv::transform_2_geometry(end_effector_pose_transform), pre_grasp_poses_->pre_grasp_poses);

    pose_publisher->publish_pose_array("pre_grasp_poses", 10, uclv::pose_stamped_2_pose_array(sorted_pre_grasp_poses));

    rclcpp::shutdown();
    return 0;
}
