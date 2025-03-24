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

        Node(
            package='odometry',
            executable='odometry'
        ),

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
            executable='transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
        ),

        # Node(
        #     package='localization',
        #     executable='stop_static',
        # ),
        Node(
            package='icp_cpp',  # The name of your package
            executable='icp_node',  # The executable of your ICP node
            name='icp_node',  # Optional: Name the node (if you want a custom name)
            output='screen',  # This will print the output to the terminal
            parameters=[{
                "mean_k_neighbours": 30,
                "std_dev_mul_thresh": 1.0,
                "max_correspondence_distance": 0.05,
                "maximum_iterations": 3000,
                "transformation_epsilon": 1e-6,
                "euclidean_fitness_epsilon": 1e-4,
                "fitness_threshold": 2.0,
            }], 
        ),



    ])