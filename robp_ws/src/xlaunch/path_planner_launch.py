from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='point_generator', #script name
            #name='point_generator', #node name when running, don't think is necessary
            parameters=['src/path_planner/config/params.yaml']
        ),
        Node(
            package='path_planner',
            executable='carrot_planner',
            #name='carrot_planner',
            parameters=['src/path_planner/config/params.yaml']
        ),

        # Check later if I'm missing anything
        Node(
            package='odometry',
            executable='odometry',
        ),

         Node(
            package='joystick_teleop',
            executable='twist2duty',
        ),
    ])