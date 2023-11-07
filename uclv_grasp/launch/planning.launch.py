import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    moveit = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('uclv_moveit_planner_ros2'), 'launch'),
         '/visualize.launch.py'])
      )

    grasp_selection_strategy_server =  Node(
            package='uclv_grasp',
            namespace='uclv_grasp',
            executable='grasp_selection_strategy_server',
            name='grasp_selection_strategy_server',
            output='screen',
            emulate_tty=True
    )

    pp_traj_server =  Node(
            package='uclv_grasp',
            namespace='uclv_grasp',
            executable='pp_traj_server',
            name='pp_traj_server',
            output='screen',
            emulate_tty=True
    )


    return LaunchDescription([
        moveit,
        grasp_selection_strategy_server,
        pp_traj_server
    ])