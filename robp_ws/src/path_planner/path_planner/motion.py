#!/usr/bin/env python3

import rclpy
import math 
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, Pose2D, Twist, PoseStamped, PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
"""
This node is used to control the robot's motion to navigate to a goal position.
PID controller was implemented to control the robot's angular velocity to reach the goal position.

You may publish a PointStamped message to the topic '/motion/goal' to set a new goal position.
When it reaches the goal, it will publish a Bool message (True) to the topic '/motion/goal_reached'.

You may also publish a Path message to the topic '/motion/path' to set a path of goal positions.
"""

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize the goal position
        self.goal_position = PoseStamped()
        self.goal_position.pose.position.x = 0.0
        self.goal_position.pose.position.y = 0.0
        self.goal_position.pose.position.z = 0.0
        self.goal_position.pose.orientation.x = 0.0
        self.goal_position.pose.orientation.y = 0.0
        self.goal_position.pose.orientation.z = 0.0
        self.goal_position.pose.orientation.w = 1.0
        self.goal_reached_flag = True

        # Setup velocity msg
        self.vel_cmd = Twist()
        self.vel_cmd.linear.y = 0.0
        self.vel_cmd.linear.z = 0.0
        self.vel_cmd.angular.x = 0.0
        self.vel_cmd.angular.y = 0.0
        self.prev_time = self.get_clock().now().nanoseconds / 1e9

        self.x_map = None
        self.y_map = None
        self.theta_map = None

        self.path_reached = False
        self.is_path = False
        self.is_goal = False
        self.do_yaw = True

        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.prev_angle_diff = 0.0


        # Setup publishers and subscribers
        # self.create_subscription(Pose2D, '/odom_pose', self.odometry_callback, 10)
        self.create_subscription(PoseStamped, '/motion/goal', self.goal_callback, 10)
        self.create_subscription(Path, '/motion/path', self.path_callback, 10)
        self.goal_reached_publisher = self.create_publisher(Bool, '/motion/goal_reached', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/motion/goal', 10)
        self.path_publisher = self.create_publisher(Path, '/motion/path', 10)
        self.path_reached_publisher = self.create_publisher(Bool, '/motion/path_reached', 10) ## TODO
        self.icp_publisher = self.create_publisher(Bool, '/icp/activate', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        # ==================
        self.linear_velocity = 0.17
        self.angular_velocity = 0.4
        self.linear_velocity_fine = 0.1 # TODO untested, adjust this value
        self.angular_velocity_fine = 0.2 # TODO untested, adjust this value
        self.goal_threshold = 0.16
        self.kp = 1.5
        self.ki = 0.015
        self.kd = 0.5
        # ==================

    # Checks when new goal is received
    def goal_callback(self, msg:PoseStamped):
        self.is_goal = True
        self.goal_position = msg
        self.goal_reached_publisher.publish(Bool(data=False))
        self.goal_reached_flag = False
        self.get_logger().info('New goal received: x={}, y={}'.format(self.goal_position.pose.position.x, self.goal_position.pose.position.y))
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.prev_angle_diff = 0.0

    def path_callback(self, msg:Path):
        if len(msg.poses) > 0:
            self.path_reached_publisher.publish(Bool(data=False))
            self.path_reached = False
            self.is_path = True
            self.path = msg
            pub_msg = PoseStamped()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "map"
            pub_msg.pose.position.x = self.path.poses[0].pose.position.x
            pub_msg.pose.position.y = self.path.poses[0].pose.position.y
            pub_msg.pose.position.z = 0.0
            pub_msg.pose.orientation.x = self.path.poses[0].pose.orientation.x
            pub_msg.pose.orientation.y = self.path.poses[0].pose.orientation.y
            pub_msg.pose.orientation.z = self.path.poses[0].pose.orientation.z
            pub_msg.pose.orientation.w = self.path.poses[0].pose.orientation.w
            self.goal_publisher.publish(pub_msg)
    
    # # Checks robot's current position and its current goal, and commands the robot to move if it hasn't reached the goal
    # def odometry_callback(self, msg: Pose2D):
    #     """ Transform the odometry pose from 'odom' to 'map' and control robot movement. """
    #     if not self.goal_reached_flag:
    #         try:
    #             # Create a PoseStamped in the odom frame
    #             pose_odom = PoseStamped()
    #             pose_odom.header.frame_id = "odom"
    #             pose_odom.header.stamp = self.get_clock().now().to_msg()
    #             pose_odom.pose.position.x = msg.x
    #             pose_odom.pose.position.y = msg.y
    #             pose_odom.pose.position.z = 0.0
    #             pose_odom.pose.orientation.w = math.cos(msg.theta / 2)
    #             pose_odom.pose.orientation.x = 0.0
    #             pose_odom.pose.orientation.y = 0.0
    #             pose_odom.pose.orientation.z = math.sin(msg.theta / 2)
    #         except Exception as e:
    #             self.get_logger().warn(f"Pose creation failed: {str(e)}")

    #         try:
    #             # Lookup transform from 'map' to 'odom'
    #             transform = self.tf_buffer.lookup_transform(
    #                 "map",  # Target frame
    #                 "odom",  # Source frame
    #                 rclpy.time.Time(),  # Get the latest available transform
    #                 timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout for lookup
    #             )
    #             # Transform the pose
    #             transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_odom.pose, transform)
    #         except Exception as e:
    #             self.get_logger().warn(f"Lookup failed: {str(e)}")

    #         try:
    #             # Extract transformed 2D position and heading
    #             self.x_map = transformed_pose.position.x
    #             self.y_map = transformed_pose.position.y
    #             qz = transformed_pose.orientation.z
    #             qw = transformed_pose.orientation.w
    #             self.theta_map = 2 * math.atan2(qz, qw)  # Convert quaternion to yaw

    #             # # Proceed with goal tracking using transformed pose
    #             # self.navigate_to_goal(self.x_map, self.y_map, self.theta_map)
    #         except Exception as e:
    #             self.get_logger().warn(f"Transform failed: {str(e)}")

    def get_robot_position(self):
        try:
            # Lookup transform from 'map' to 'base_link'
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'base_link',  # Source frame
                rclpy.time.Time(),  # Get the latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout for lookup
            )
            # Extract transformed 2D position and heading
            self.x_map = transform.transform.translation.x
            self.y_map = transform.transform.translation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            self.theta_map = 2 * math.atan2(qz, qw)  # Convert quaternion to yaw
        except Exception as e:
            self.get_logger().warn(f"Lookup failed: {str(e)}")
            return False
        return True

    def navigate_to_goal(self, fine=False):
        """ Control the robot to navigate to the goal position. """
        self.get_robot_position()
        x = self.x_map
        y = self.y_map
        theta = self.theta_map

        goal_x = self.goal_position.pose.position.x
        goal_y = self.goal_position.pose.position.y

        qz = self.goal_position.pose.orientation.z
        qw = self.goal_position.pose.orientation.w
        self.angle_goal = 2 * math.atan2(qz, qw)

        if x is None or y is None or theta is None: return False

        distance = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
        angle = math.atan2(goal_y - y, goal_x - x)
        # Normalize angle difference to range [-pi, pi]
        angle_diff = math.atan2(math.sin(angle - theta), math.cos(angle - theta))
        # angle_diff = angle - theta
        cur_time = self.get_clock().now().nanoseconds / 1e9
        self.elapsed_time = cur_time - self.prev_time
        
        if distance > self.goal_threshold:
            iError = angle_diff * self.elapsed_time
            dError = (angle_diff - self.prev_angle_diff)/self.elapsed_time
            output = self.kp*angle_diff + self.ki*iError + self.kd*dError
            if output > self.angular_velocity if not fine else self.angular_velocity_fine:
                output = self.angular_velocity if not fine else self.angular_velocity_fine
            elif output < -self.angular_velocity if not fine else self.angular_velocity_fine:
                output = -self.angular_velocity if not fine else self.angular_velocity_fine
            self.vel_cmd.angular.z = output
            if angle_diff > math.pi/7 or angle_diff < -math.pi/7: # TODO adjust this value
                self.vel_cmd.linear.x = 0.0
            else:
                self.vel_cmd.linear.x = self.linear_velocity if not fine else self.linear_velocity_fine
            self.cmd_vel_publisher.publish(self.vel_cmd)
        else:
            self.goal_reached_publisher.publish(Bool(data=True))
            self.goal_reached_flag = True
            self.get_logger().info('Goal reached: x={}, y={}'.format(goal_x, goal_y))
            
            # self.is_goal = False
            if self.is_path and len(self.path.poses) >= 1:
                self.path.poses.pop(0)
                self.path_publisher.publish(self.path)
            
            elif self.is_path and len(self.path.poses) == 0:
                self.get_logger().info('Path execution completed.')
                self.path_reached_publisher.publish(Bool(data=True))
                self.path_reached = True
                # self.is_path = False
                self.icp_publisher.publish(Bool(data=False))
                self.vel_cmd.angular.z = 0.0
                self.vel_cmd.linear.x = 0.0
                self.cmd_vel_publisher.publish(self.vel_cmd)
                if self.do_yaw: self.adjust_yaw(self.angle_goal)
            else:
                self.vel_cmd.angular.z = 0.0
                self.vel_cmd.linear.x = 0.0
                self.cmd_vel_publisher.publish(self.vel_cmd)
                #if self.do_yaw: self.adjust_yaw(self.angle_goal) #TODO: improve adjust yaw
        
        self.prev_angle_diff = angle_diff
        self.prev_time = self.get_clock().now().nanoseconds / 1e9

        return True
    
    def adjust_yaw(self, angle):
        while True:
            self.get_robot_position()
            x = self.x_map
            y = self.y_map
            theta = self.theta_map

            angle_diff = math.atan2(math.sin(angle - theta), math.cos(angle - theta))
            self.get_logger().info(f'Angle difference is {angle_diff}')
            if abs(angle_diff) < 0.1:
                break
            if angle_diff > 0:
                self.vel_cmd.angular.z = self.angular_velocity_fine
            else:
                self.vel_cmd.angular.z = -self.angular_velocity_fine
            self.vel_cmd.linear.x = 0.0
            self.cmd_vel_publisher.publish(self.vel_cmd)
            
        self.vel_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.vel_cmd)
        return

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()