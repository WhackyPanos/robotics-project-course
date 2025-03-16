from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robp_launch_dir = get_package_share_directory('robp_launch')

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/rs_d435i_launch.py'))
        # ),
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
            package='detection',
            executable='clustering_node',
            output='screen',
            parameters=[{
                "cloud_topic": "/camera/camera/depth/color/points",
                "cluster_topic": "/detection/cluster_points",
                "map_topic": "/map",
                "twist_topic": "/cmd_vel",
                "dist_filter_min": 0.0,
                "dist_filter_max": 1.0,
                "height_filter_min": -0.025,
                "height_filter_max": 0.075,
                "cluster_tolerance": 0.05,
                "cluster_min_size": 100,
                "occupancy_margin": 2,
                "occupancy_value": 99,
                "ang_vel_threshold": 0.0
            }]
        ),
        Node(
            package='detection',
            executable='classifier_node',
            output='screen',
            parameters=[{
                "cloud_topic": "/detection/cluster_points",
                "classification_topic": "/detection/class",
                "twist_topic": "/cmd_vel",
                "box_filter_min": 0.0,
                "box_filter_max": 0.0069,
                "box_filter_threshold": 50,
                "animal_filter_min": 0.045,
                "animal_filter_max": 0.048,
                "sphere_filter_min": 0.056,
                "sphere_filter_max": 0.059,
                "ang_vel_threshold": 0.0,
                "lin_vel_threshold": 0.0,
                "visualize_OBB": True
            }]
        ),
         Node(
            package='map_file',
            executable='map_file',
            output='screen',
            parameters=[{
                'box_threshold': 20,
                'object_threshold': 7,
                'msg_topic': '/detection/class'
            }],
        )
    ])