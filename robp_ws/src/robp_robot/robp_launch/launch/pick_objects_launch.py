import launch
from launch_ros.actions import Node
from launch_ros.actions import Node

def generate_launch_description():

    return launch.LaunchDescription([
        Node(
            package='behavior_tree',
            executable='pick_bt',
            output='screen',   # This ensures output goes to the terminal
            emulate_tty=True   # Ensures that print() statements are immediately flushed
        )
    ])
