import launch
from launch_ros.actions import Node
import os
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robp_launch_dir = get_package_share_directory('robp_launch')
    return launch.LaunchDescription([

        # Node(
        #     package='odometry',
        #     executable='odometry'
        # ),

        Node(
            package='behavior_tree',
            executable='collection_BT'
        ),

        Node(
            package='joystick_teleop',
            executable='twist2duty'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
        ),

    ])