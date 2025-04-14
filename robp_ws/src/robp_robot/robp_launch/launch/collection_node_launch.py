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
        #     package='icp_cpp',  # The name of your package
        #     executable='icp_node',  # The executable of your ICP node
        #     name='icp_node',  
        #     output='screen',  
        #     parameters=[{
        #         #"use_sim_time": True,
        #         "transform_lookup_timeout": 2.0,
        #         "KNN_N_neighbours": 20,
        #         "std_dev_mul_thresh": 0.8,
        #         "max_correspondence_distance": 0.005,
        #         "maximum_iterations": 5000,
        #         "transformation_epsilon": 1e-9,
        #         "euclidean_fitness_epsilon": 1e-4,
        #         "icp_fitness_threshold": 2.0
        #     }]
        # ),

        # Node(
        #     package='localization',  
        #     executable='localization_transform',  
        #     name='localization_transform',  
        #     output='screen',  
        #     parameters=[{
        #         #"use_sim_time": True,
        #         "transform_lookup_timeout": 2.0
        #     }]
        # ),
        Node(
            package='arm_cam',
            executable='object_segmentation_node'
        ),

        Node(
            package='mapping',
            executable='occupancy_grid_collection'
        ),

        # Node(
        #     package='behavior_tree',
        #     executable='collection_BT' #collection_BT or collection_BT_no_move, but no_move it is not working
        # ),

        # Node(
        #     package='joystick_teleop',
        #     executable='twist2duty'
        # ),



    ])