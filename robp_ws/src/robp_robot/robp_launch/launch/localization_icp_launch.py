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
            package='path_planner',
            executable='point_generator', #script name
            #name='point_generator', #node name when running, don't think is necessary
            #parameters=['src/path_planner/config/params.yaml']
        ),
        Node(
            package='path_planner',
            executable='path_planner',
            #name='carrot_planner',
            #parameters=['src/path_planner/config/params.yaml']
        ),

        Node(
            package='joystick_teleop',
            executable='twist2duty'
        ),

        Node(
            package='icp_cpp',  # The name of your package
            executable='icp_node',  # The executable of your ICP node
            name='icp_node',  # Optional: Name the node (if you want a custom name)
            output='screen',  # This will print the output to the terminal
            parameters=[],  # Optional: Add any parameters you want to pass to the node
        )


        # # RViz Node (Loads a specific RViz config file if available) possibility to add rviz specification
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', os.path.join(robp_launch_dir, 'config', 'lidar_config.rviz')]
        # ),
    ])


