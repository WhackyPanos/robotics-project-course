#!/usr/bin/env python
# stuff
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Header
from robp_interfaces.msg import DutyCycles
from geometry_msgs.msg import Twist
#from math import atan2, sin, sqrt, cos


class F710Teleop(Node):
    def __init__(self):
        super().__init__('f710_teleop')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Define joystick mappings (adjust based on testing)    
        self.axis_straight = 4  # Left stick up/down
        self.axis_sideways = 3  # Left stick left/right
        self.rotate_button = 4  # CHANGE LATER ON
        self.max_linear = 0.08 # maximum linear velocity
        self.max_angular = 0.35 # maximum angular velocity

        # Add a timer delay on startup
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        self.timer.cancel()  # Cancel the timer after it runs once

    def joy_callback(self, msg):
        # Print joystick values
        print(f'Up/down (button {self.axis_straight}): {msg.axes[self.axis_straight]}')
        print(f'Left/right (button {self.axis_sideways}): {msg.axes[self.axis_sideways]}')

        # Publish twist commands
        twist_msg = Twist()

        #theta = atan2(msg.axes[self.axis_straight], msg.axes[self.axis_sideways])
        #vx = sin(theta)*sqrt(msg.axes[self.axis_straight]**2 + msg.axes[self.axis_sideways]**2)*(1/sqrt(2))*self.max_linear
        #vx = cos(theta)*sqrt(msg.axes[self.axis_straight]**2 + msg.axes[self.axis_sideways]**2)*(1/sqrt(2))*self.max_linear

        twist_msg.linear.x = msg.axes[self.axis_straight] * self.max_linear
        if msg.buttons[self.rotate_button] == 1 or msg.buttons[self.rotate_button] == 2: # LB
            twist_msg.angular.z = msg.axes[self.axis_sideways] * self.max_angular
            twist_msg.linear.x = 0.0
        else:
            twist_msg.angular.z = 0.0

        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        # print(f'Linear Velocity (x,y): {twist_msg.linear.x} and {twist_msg.linear.y}')
        # self.get_logger().info(f'Angular Velocity: {twist_msg.angular.z}')

        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = F710Teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()