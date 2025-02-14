import launch #from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/hiwonder_arm', '-v6'],
            output='screen'
        ),

        Node(
            package='pick_objects',
            executable='pick_objects')  #script name
    ])


#ros2 run arm_controller arm_controller 