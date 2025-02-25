#!/usr/bin/env python

import rclpy
import math
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped, TransformStamped, PointStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy



class CollectObjectMS2(Node):
    def __init__(self):
        super().__init__('collect_object_ms2')

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rate = self.create_rate(50)

        # Initialize the goal position
        self.goal_position = Point()
        self.goal_position.x = 0.0
        self.goal_position.y = 0.0
        self.goal_reached_flag = True

        self.vel_cmd = Twist()
        self.vel_cmd.linear.y = 0.0
        self.vel_cmd.linear.z = 0.0
        self.vel_cmd.angular.x = 0.0
        self.vel_cmd.angular.y = 0.0
        
        self.x_map = 0.0
        self.y_map = 0.0
        self.theta_map = 0.0

        self.avoiding_obstacle = False
        self.obstacle_found = False

        #self.create_subscription(PoseStamped, '/path', self.odometry_callback, 10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Pose2D, '/odom_pose', self.odometry_callback, qos_profile)
        self.create_subscription(PointStamped, '/temp_goal', self.goal_callback, qos_profile)
        self.goal_reached_publisher = self.create_publisher(Bool, '/goal_reached', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Retrieve parameter values
        self.linear_velocity = 0.1
        self.angular_velocity = 0.2
        self.goal_threshold = 0.14

    def goal_callback(self, msg: PointStamped):
        """ Callback function to receive the goal position from the behavior tree. """
        self.get_logger().info("Goal Received!")
        self.goal_position.x = msg.point.x
        self.goal_position.y = msg.point.y
        self.goal_reached_flag = False
        self.get_logger().info(f"New goal received: x={msg.point.x:.2f}, y={msg.point.y:.2f}")
        self.mline = (self.goal_position.y - self.y_map)/(self.goal_position.x - self.x_map) if (self.goal_position.x - self.x_map) !=0 else float('inf')


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
                pose_odom.pose.orientation.z = math.sin(msg.theta / 2)
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
                self.x_map = transformed_pose.position.x
                self.y_map = transformed_pose.position.y
                qz = transformed_pose.orientation.z
                qw = transformed_pose.orientation.w
                self.theta_map = 2 * math.atan2(qz, qw)  # Convert quaternion to yaw

                # Log transformed pose
                #self.get_logger().info(f"Transformed Pose in 'map' frame: x={x_map:.2f}, y={y_map:.2f}, theta={theta_map:.2f}")

                # Proceed with goal tracking using transformed pose
                self.navigate_to_goal(self.x_map, self.y_map, self.theta_map)

            except Exception as e:
                self.get_logger().warn(f"Transform failed: {str(e)}")

    def navigate_to_goal(self, x, y, theta):
        """ Compute commands to move the robot towards the goal in map frame. """
        """ Parameters """
        angle_tolerance = 0.1  # Angle threshold for facing the goal (adjust as needed)
        distance_to_goal = math.sqrt((self.goal_position.x - x) ** 2 + (self.goal_position.y - y) ** 2)
        goal_angle = math.atan2(self.goal_position.y - y, self.goal_position.x - x)
        # Check if the goal is behind
        robot_forward_x = math.cos(theta)
        robot_forward_y = math.sin(theta)
        to_goal_x = self.goal_position.x - x
        to_goal_y = self.goal_position.y - y
        
        self.curr_line = (y)/(x) if x != 0 else float('inf')

        dot_product = (robot_forward_x * to_goal_x) + (robot_forward_y * to_goal_y)

        angle_difference = (goal_angle - theta + math.pi) % (2 * math.pi) - math.pi
        # self.get_logger().info('Angle diff: ' + str(angle_difference))
        # self.get_logger().info('Distance: ' + str(distance_to_goal))
        # self.get_logger().info('My angle: ' + str(theta))

        if distance_to_goal > self.goal_threshold:

            if dot_product < 0:  # If the goal is behind, rotate in place
                self.vel_cmd.linear.x = 0.0  # Don't move forward yet
                self.vel_cmd.angular.z = self.angular_velocity if angle_difference > 0 else -1.0  # Rotate in place
            else:
                if abs(angle_difference) > angle_tolerance:  
                    self.vel_cmd.linear.x = 0.0  # Stop forward motion while turning
                    self.vel_cmd.angular.z = self.angular_velocity if angle_difference > 0 else -1.0  # Rotate towards the goal
                else:
                    self.vel_cmd.linear.x = min(self.linear_velocity, self.linear_velocity * distance_to_goal * 5)
                    self.vel_cmd.angular.z = 0.0  # Move forward when facing the goal
        else:
            self.stop()
            self.goal_reached_publisher.publish(Bool(data=True))
            self.goal_reached_flag = True
            self.get_logger().info("Goal reached!")
            return

        self.cmd_vel_publisher.publish(self.vel_cmd)
        self.rate.sleep()

            
    def stop(self):
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.vel_cmd)


