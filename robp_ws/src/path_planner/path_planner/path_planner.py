#!/usr/bin/env python

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
#from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D, PoseStamped
#import tf_transformations

class CarrotPlanner(Node):
    def __init__(self):
        super().__init__('carrot_planner')

       # Initialize the goal position
        self.goal_position = Point()
        self.goal_position.x = 0.0
        self.goal_position.y = 0.0
        self.goal_reached_flag = True
 
        #self.create_subscription(PoseStamped, '/path', self.odometry_callback, 10)
        self.create_subscription(Pose2D, '/odom_pose', self.odometry_callback, 10)
        self.create_subscription(Point, '/temp_goal', self.goal_callback, 10)
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
        self.create_timer(0.5, self.publish_initial_goal)
        #self.publish_initial_goal()

    def publish_initial_goal(self):
        """Publish a True message on /goal_reached to trigger the first point generation."""
        self.get_logger().info("Publishing initial goal reached message.")
        self.goal_reached_publisher.publish(Bool(data=True))

    def goal_callback(self, msg: Point):
        self.goal_position.x = msg.x
        self.goal_position.y = msg.y
        self.goal_reached_publisher.publish(Bool(data=False))
        self.goal_reached_flag = False
        self.get_logger().info(f"New goal received: x={msg.x:.2f}, y={msg.y:.2f}")

    def odometry_callback(self, msg: Pose2D):
        """ This functon uses the odometry data directly (created a topic in the odometry package)"""
        if not self.goal_reached_flag:
            goal_angle = math.atan2(self.goal_position.y - msg.y, 
                                    self.goal_position.x - msg.x)
            robot_heading = msg.theta
            angle_difference = goal_angle - robot_heading # we might need to normalize this

            if abs(angle_difference) >= 0.01:
                rotate_command = Twist()
                rotate_command.angular.z = self.angular_velocity
                self.cmd_vel_publisher.publish(rotate_command)
            else:
                rotate_command = Twist()
                rotate_command.angular.z = 0.0
                self.cmd_vel_publisher.publish(rotate_command)

                distance_to_goal = math.sqrt((self.goal_position.x - msg.x) ** 2 + (self.goal_position.y - msg.y) ** 2)

                if distance_to_goal >= self.goal_threshold:
                    move_command = Twist()
                    move_command.linear.x = self.linear_velocity
                    self.cmd_vel_publisher.publish(move_command)
                else:
                    move_command = Twist()
                    move_command.linear.x = 0.0
                    self.cmd_vel_publisher.publish(move_command)
                    self.goal_reached_publisher.publish(Bool(data=True))
                    self.goal_reached_flag = True
                    self.get_logger().info("Goal reached!")
    """
    def odometry_callback(self, msg: PoseStamped):
        if not self.goal_reached_flag:
            goal_angle = math.atan2(self.goal_position.y - msg.pose.position.y, 
                                    self.goal_position.x - msg.pose.position.x)
            qz = msg.pose.orientation.z

            [_, _, robot_heading] = tf_transformations.euler_from_quaternion([0, 0, qz, 1])

            angle_difference = goal_angle - robot_heading
            if abs(angle_difference) >= 0.01:
                rotate_command = Twist()
                rotate_command.angular.z = self.angular_velocity
                self.cmd_vel_publisher.publish(rotate_command)
            else:
                rotate_command = Twist()
                rotate_command.angular.z = 0.0
                self.cmd_vel_publisher.publish(rotate_command)

                distance_to_goal = math.sqrt((self.goal_position.x - msg.pose.position.x) ** 2 + 
                                             (self.goal_position.y - msg.pose.position.y) ** 2)
                if distance_to_goal >= self.goal_threshold:
                    move_command = Twist()
                    move_command.linear.x = self.linear_velocity
                    self.cmd_vel_publisher.publish(move_command)
                else:
                    move_command = Twist()
                    move_command.linear.x = 0.0
                    self.cmd_vel_publisher.publish(move_command)
                    self.goal_reached_publisher.publish(Bool(data=True))
                    self.goal_reached_flag = True
                    self.get_logger().info("Goal reached!")
    """

    
def main(args=None):
    rclpy.init(args=args)
    node = CarrotPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
