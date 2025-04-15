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

        Node(
            package='detection',
            executable='clustering_collection_node',
            output='screen',
            parameters=[{
                "cloud_topic": "/camera/camera/depth/color/points",
                "map_topic": "/occupancy_grid",
                "twist_topic": "/cmd_vel",
                "trigger_topic": "/detection/request",
                "result_topic": "/detection/result",
                "dist_filter_min": 0.2,
                "dist_filter_max": 1.0,
                "height_filter_min": -0.03,
                "height_filter_max": 0.072,
                "cluster_tolerance": 0.05,
                "cluster_min_size": 100,
                "occupancy_margin": 0,
                "occupancy_value": 0
                }]
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