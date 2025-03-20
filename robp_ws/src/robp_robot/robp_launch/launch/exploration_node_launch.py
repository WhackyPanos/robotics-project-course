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
        #     name='icp_node',  # Optional: Name the node (if you want a custom name)
        #     output='screen',  # This will print the output to the terminal
        #     parameters=[],  # Optional: Add any parameters you want to pass to the node
        # ),

        # Node(
        #     package='localization',  # The name of your package
        #     executable='ekf',  # The executable of your ICP node
        #     name='ekf',  # Optional: Name the node (if you want a custom name)
        #     output='screen',  # This will print the output to the terminal
        #     parameters=[],  # Optional: Add any parameters you want to pass to the node
        # ),

        Node(
            package='odometry',
            executable='odometry'
        ),

        Node(
            package='behavior_tree',
            executable='exploration_BT'
        ),

        # Node(
        #     package='detection',
        #     executable='clustering_node',
        #     output='screen',
        #     parameters=[{
        #         "cloud_topic": "/camera/camera/depth/color/points",
        #         "cluster_topic": "/detection/cluster_points",
        #         "map_topic": "/map",
        #         "twist_topic": "/cmd_vel",
        #         "trigger_topic": "/detection/request",
        #         "result_topic": "/detection/result",
        #         "dist_filter_min": 0.0,
        #         "dist_filter_max": 1.0,
        #         "height_filter_min": -0.025,
        #         "height_filter_max": 0.075,
        #         "cluster_tolerance": 0.05,
        #         "cluster_min_size": 100,
        #         "occupancy_margin": 1,
        #         "occupancy_value": 1,
        #         "ang_vel_threshold": 0.0
        #         }]
        #     ),
        # Node(
        #     package='detection',
        #     executable='classifier_node',
        #     output='screen',
        #     parameters=[{
        #         "cloud_topic": "/detection/cluster_points",
        #         "classification_topic": "/classification/class",
        #         "twist_topic": "/cmd_vel",
        #         "trigger_topic_": "/classification/request",
        #         "result_topic_": "/classification/result",
        #         "box_filter_min": 0.0,
        #         "box_filter_max": 0.0069,
        #         "box_filter_threshold": 50,
        #         "animal_filter_min": 0.045,
        #         "animal_filter_max": 0.048,
        #         "sphere_filter_min": 0.056,
        #         "sphere_filter_max": 0.059,
        #         "ang_vel_threshold": 0.0,
        #         "lin_vel_threshold": 0.0,
        #         "visualize_OBB": True
        #     }]
        # ),

        # Node(
        #     package='map_file',
        #     executable='map_file',
        #     output='screen',
        #     parameters=[{
        #         'box_threshold': 20,
        #         'object_threshold': 7,
        #         'msg_topic': '/classification/class'
        #     }]
        # )
    ])