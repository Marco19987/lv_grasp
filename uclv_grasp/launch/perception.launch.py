import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    realsense = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('lv_realsense'), 'launch'),
         '/start_realsense.launch.py'])
      )

    dope = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('dope_ros2'), 'launch'),
         '/dope.launch.py'])
      )
    
    # it needs to be executed within the conda environmentss
    # depth_optimizer_server = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('depth_optimization'), 'launch'),
    #      '/depth_optimizer.launch.py'])
    #   )
        
    pose_post_proc_server =  Node(
            package='uclv_grasp',
            namespace='uclv_grasp',
            executable='pose_post_proc_server',
            name='pose_post_proc_server',
            output='screen',
            emulate_tty=True
    )

    return LaunchDescription([
        realsense,
        dope,
        #depth_optimizer_server,
        pose_post_proc_server
    ])