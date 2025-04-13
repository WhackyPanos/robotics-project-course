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
                "KNN_N_neighbours": 50,
                "std_dev_mul_thresh": 10.0,
                "max_correspondence_distance": 0.05,  # Maximum distance between corresponding points in source and target clouds
                "maximum_iterations": 20000,  # Maximum number of iterations for ICP to refine alignment
                "transformation_epsilon": 1e-6,  # Convergence criterion: minimum change in transformation between iterations
                "euclidean_fitness_epsilon": 1e-4,  # Convergence criterion: minimum mean squared error change between iterations
                "icp_fitness_threshold": 1.0  # Threshold to accept the final ICP alignment based on fitness score
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

        #NOTE: To record rosbags:
        # ros2 launch robp_launch localization_icp_launch.py 
        # ros2 bag record /tf /tf_static /rosout /scan /odom_pose /parameter_events

        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/lidar_launch.yaml'))
        # ),


        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/frames_launch.xml'))
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(robp_launch_dir, 'launch/phidgets_launch.py'))
        # ),

        # Node(
        #     package='odometry',
        #     executable='odometry'
        # ),

        # Node(
        #     package='joystick_teleop',
        #     executable='teleop'
            
        # ),

        #   Node(
        #     package='joystick_teleop',
        #     executable='twist2duty'
            
        # ),
        # Node(
        #     package='joy',
        #     executable='joy_node'
        # ),
        # Node(
        #     package='odometry',
        #     executable='odometry'
        # )
        

    ])


