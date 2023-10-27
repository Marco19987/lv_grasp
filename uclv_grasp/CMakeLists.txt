cmake_minimum_required(VERSION 3.8)
project(uclv_grasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uclv_grasp_interfaces REQUIRED)
find_package(uclv_moveit_planner_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depth_optimization_interfaces REQUIRED)
find_package(uclv_utilities REQUIRED)
find_package(uclv_moveit_planner_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_executable(pose_post_proc_server src/pose_post_proc_server.cpp)
ament_target_dependencies(pose_post_proc_server rclcpp 
                          uclv_grasp_interfaces geometry_msgs 
                          sensor_msgs std_msgs tf2_ros cv_bridge 
                          depth_optimization_interfaces uclv_utilities)

add_executable(srv_post_proc_client src/srv_post_proc_client.cpp)
ament_target_dependencies(srv_post_proc_client rclcpp uclv_grasp_interfaces std_msgs)

add_executable(pp_traj_server src/pp_traj_server.cpp)
ament_target_dependencies(pp_traj_server rclcpp geometry_msgs moveit_ros_planning_interface uclv_utilities uclv_moveit_planner_interface uclv_moveit_planner_ros2 uclv_grasp_interfaces)

add_executable(test_pp_client src/test_pp_client.cpp)
ament_target_dependencies(test_pp_client rclcpp geometry_msgs moveit_ros_planning_interface uclv_utilities uclv_moveit_planner_interface uclv_grasp_interfaces)

# add_executable(grasp_selection_strategy_server src/grasp_selection_strategy_server.cpp)
# ament_target_dependencies(grasp_selection_strategy_server rclcpp uclv_grasp_interfaces std_msgs geometry_msgs)

install(TARGETS
  pose_post_proc_server 
  srv_post_proc_client
  # grasp_selection_strategy_server
  pp_traj_server
  test_pp_client
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


