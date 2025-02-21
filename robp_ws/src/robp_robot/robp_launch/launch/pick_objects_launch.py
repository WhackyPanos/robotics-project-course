import launch
from launch_ros.actions import Node
import os

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='behavior_tree',
            executable='pick_bt',
            output='screen',   # This ensures output goes to the terminal
            emulate_tty=True   # Ensures that print() statements are immediately flushed
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
        ),
        Node(
            package='odometry',
            executable='odometry'
        ),
    ])
