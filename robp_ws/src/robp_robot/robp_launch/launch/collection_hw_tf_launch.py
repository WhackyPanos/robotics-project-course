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
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/lidar_launch.yaml'))
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/arm_camera_launch.yaml'))
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/rs_d435i_launch.py'))
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/frames_launch.xml'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/phidgets_launch.py'))
        ),

        Node(
            package='odometry',
            executable='odometry'
        ),


    ])