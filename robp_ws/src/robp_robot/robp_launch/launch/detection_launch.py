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
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/phidgets_launch.py'))
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
        # Node(
        #     package='detection',
        #     executable='detection'
        # ),
        Node(
            package='detection',
            executable='clustering_node',
            output='screen'
        ),
        Node(
            package='detection',
            executable='classifier_node',
            output='screen'
        ),
         Node(
            package='map_file',
            executable='map_file'
        )
    ])