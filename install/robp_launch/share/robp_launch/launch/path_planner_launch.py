#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # call the odometry launch file
    robp_launch_dir = get_package_share_directory('robp_launch')


    return LaunchDescription([
        Node(
            package='path_planner',
            executable='point_generator', #script name
            #name='point_generator', #node name when running, don't think is necessary
            parameters=['src/path_planner/config/params.yaml']
        ),
        Node(
            package='path_planner',
            executable='path_planner',
            #name='carrot_planner',
            parameters=['src/path_planner/config/params.yaml']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
        ),

        # Check later if I'm missing anything
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/joystick_launch.py'))
        ),
    ])