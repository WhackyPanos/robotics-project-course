from launch_ros.actions import Node
import launch

def generate_launch_description():

    return launch.LaunchDescription([
         Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=["serial", "--dev", "/dev/hiwonder_arm", "-v6"]
        )

    ])


