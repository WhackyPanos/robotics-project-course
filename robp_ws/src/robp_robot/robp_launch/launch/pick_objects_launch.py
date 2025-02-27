from launch_ros.actions import Node
import launch
from launch_ros.actions import Node
import os
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robp_launch_dir = get_package_share_directory('robp_launch')
    # Get the current directory (where the launch file is located)
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Define the path to the custom .rviz file in the same directory
    rviz_config_path = os.path.join(launch_dir, 'rviz2_collection.rviz')  # Adjust the filename if needed
    
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/phidgets_launch.py'))
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/frames_launch.xml'))
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/lidar_launch.yaml'))
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
            package='behavior_tree',
            executable='pick_bt',
            output='screen',   # This ensures output goes to the terminal
            emulate_tty=True   # Ensures that print() statements are immediately flushed
        ),
        Node(
            package='joystick_teleop',
            executable='twist2duty'
            
        ),

        #  Node(
        #     package='robp_launch',
        #     executable='rviz2',
        #     output='screen',
        #     arguments = ['-d',rviz_config_path]

            # arguments=['-d', os.path.join(
            #     get_package_share_directory('your_package_name'),  # Replace with your package name
            #     'rviz', 'default.rviz') ] # Path to your .rviz configuration file
        #)
    ])


