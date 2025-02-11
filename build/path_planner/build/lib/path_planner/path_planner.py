#!/usr/bin/env python

import rclpy
import math
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped, TransformStamped, PointStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

class CarrotPlanner(Node):
    def __init__(self):
        super().__init__('carrot_planner')

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

       # Initialize the goal position
        self.goal_position = Point()
        self.goal_position.x = 0.0
        self.goal_position.y = 0.0
        self.goal_reached_flag = True
 
        #self.create_subscription(PoseStamped, '/path', self.odometry_callback, 10)
        self.create_subscription(Pose2D, '/odom_pose', self.odometry_callback, 10)
        self.create_subscription(PointStamped, '/temp_goal', self.goal_callback, 10)
        self.goal_reached_publisher = self.create_publisher(Bool, '/goal_reached', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Declare parameters with default values. If the parameters are set in the launch file, the default values will be overwritten
        # check config folder for params.yaml
        self.declare_parameter('linear_velocity', 0.01)
        self.declare_parameter('angular_velocity', 0.005)
        self.declare_parameter('goal_threshold', 0.1)

        # Retrieve parameter values
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.goal_threshold = self.get_parameter('goal_threshold').value

        # Initial message to trigger the first point generation
        self.timer = self.create_timer(4, self.publish_initial_goal)
        #self.publish_initial_goal()

    def publish_initial_goal(self):
        """Publish a True message on /goal_reached to trigger the first point generation."""
        self.get_logger().info("Publishing initial goal reached message.")
        self.goal_reached_publisher.publish(Bool(data=True))
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None  # Reset the timer reference

    def goal_callback(self, msg: PointStamped):
        self.goal_position.x = msg.point.x
        self.goal_position.y = msg.point.y
        self.goal_reached_publisher.publish(Bool(data=False))
        self.goal_reached_flag = False
        self.get_logger().info(f"New goal received: x={msg.point.x:.2f}, y={msg.point.y:.2f}")

    def odometry_callback(self, msg: Pose2D):
        """ Transform the odometry pose from 'odom' to 'map' and control robot movement. """
        if not self.goal_reached_flag:
            try:
                # Create a PoseStamped in the odom frame
                pose_odom = PoseStamped()
                pose_odom.header.frame_id = "odom"
                pose_odom.header.stamp = self.get_clock().now().to_msg()
                pose_odom.pose.position.x = msg.x
                pose_odom.pose.position.y = msg.y
                pose_odom.pose.position.z = 0.0
                pose_odom.pose.orientation.w = math.cos(msg.theta / 2)
                pose_odom.pose.orientation.x = 0.0
                pose_odom.pose.orientation.y = 0.0
                pose_odom.pose.orientation.z = math.sin(msg.theta / 2) #third component of quaternion, since it's yaw
            except Exception as e:
                self.get_logger().warn(f"Pose creation failed: {str(e)}")

            try:
                # Lookup transform from 'map' to 'odom'
                transform = self.tf_buffer.lookup_transform(
                    "map",  # Target frame
                    "odom",  # Source frame
                    rclpy.time.Time(),  # Get the latest available transform
                    timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout for lookup
                )
                # Transform the pose
                transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_odom.pose, transform)
            except Exception as e:
                self.get_logger().warn(f"Lookup failed: {str(e)}")
                

            try:
                # Extract transformed 2D position and heading
                x_map = transformed_pose.position.x
                y_map = transformed_pose.position.y
                qz = transformed_pose.orientation.z
                qw = transformed_pose.orientation.w
                theta_map = 2 * math.atan2(qz, qw)  # Convert quaternion to yaw

                # Log transformed pose
                self.get_logger().info(f"Transformed Pose in 'map' frame: x={x_map:.2f}, y={y_map:.2f}, theta={theta_map:.2f}")

                # Proceed with goal tracking using transformed pose
                self.navigate_to_goal(x_map, y_map, theta_map)

            except Exception as e:
                self.get_logger().warn(f"Transform failed: {str(e)}")

    def navigate_to_goal(self, x, y, theta):
        """ Compute commands to move the robot towards the goal in map frame. """
        goal_angle = math.atan2(self.goal_position.y - y, self.goal_position.x - x)
        angle_difference = goal_angle - theta
        self.get_logger().info('Angle diff: ' + str(angle_difference))

        twist_command = Twist()

        if abs(angle_difference) >= 0.3:
            twist_command.angular.z = self.angular_velocity # this angular velocity is the one from the param.yaml (maximum)
            self.cmd_vel_publisher.publish(twist_command)

        else:
            distance_to_goal = math.sqrt((self.goal_position.x - x) ** 2 + (self.goal_position.y - y) ** 2)
            if distance_to_goal >= self.goal_threshold:
                twist_command.linear.x = self.linear_velocity # again, max linear velocity is assumed
                self.cmd_vel_publisher.publish(twist_command)
            else:
                twist_command.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist_command)
                self.goal_reached_publisher.publish(Bool(data=True))
                self.goal_reached_flag = True
                self.get_logger().info("Goal reached!")

    
def main(args=None):
    rclpy.init(args=args)
    node = CarrotPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
