from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robp_launch_dir = get_package_share_directory('robp_launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/phidgets_launch.py'))
        ),

        
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/frames_launch.xml'))
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
        
        Node(
            package='joystick_teleop',
            executable='teleop'
            
        ),
        # added this node (Francisco)
          Node(
            package='joystick_teleop',
            executable='twist2duty'
            
        ),
        Node(
            package='joy',
            executable='joy_node'
        ),

        
    ])

        
  