from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robp_launch_dir = get_package_share_directory('robp_launch')

    return LaunchDescription([
        Node(
            package='icp_cpp',  # The name of your package
            executable='icp_node',  # The executable of your ICP node
            name='icp_node',  
            output='screen',  
            parameters=[{
                "use_sim_time": True,
                "transform_lookup_timeout": 2.0,
                "KNN_N_neighbours": 20,
                "std_dev_mul_thresh": 0.8,
                "max_correspondence_distance": 0.005,
                "maximum_iterations": 5000,
                "transformation_epsilon": 1e-9,
                "euclidean_fitness_epsilon": 1e-4,
                "icp_fitness_threshold": 2.0
            }]
        ),

        Node(
            package='localization',  
            executable='localization_transform',  
            name='localization_transform',  
            output='screen',  
            parameters=[{
                "use_sim_time": True,
                "transform_lookup_timeout": 2.0
            }]
        ),


        # # RViz Node (Loads a specific RViz config file if available) possibility to add rviz specification
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', os.path.join(robp_launch_dir, 'config', 'lidar_config.rviz')]
        # ),
    ])


