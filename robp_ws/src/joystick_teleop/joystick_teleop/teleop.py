#!/usr/bin/env python
# stuff
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Header
from robp_interfaces.msg import DutyCycles
from geometry_msgs.msg import Twist


class F710Teleop(Node):
    def __init__(self):
        super().__init__('f710_teleop')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, '/desired_twist', 10)

        # Define joystick mappings (adjust based on testing)    
        self.axis_linear = 7  # Left stick up/down
        self.axis_angular = 6  # Left stick left/right
        self.max_linear = 1.0 # maximum linear velocity
        self.max_angular = 0.5 # maximum angular velocity

    def joy_callback(self, msg):
        # Print joystick values
        print(f'Up/down (button {self.axis_linear}): {msg.axes[self.axis_linear]}')
        print(f'Left/right (button {self.axis_angular}): {msg.axes[self.axis_angular]}')

        # Publish twist commands
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        """
        msg.linear.x = linear
        msg.linear.y = 0.0
        msg.angular.z = angular
        """
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = F710Teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()