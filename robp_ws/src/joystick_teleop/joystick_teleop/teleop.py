#!/usr/bin/env python
# stuff
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Header
from robp_interfaces.msg import DutyCycles

class F710Teleop(Node):
    def __init__(self):
        super().__init__('f710_teleop')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)

        # Define joystick mappings (adjust based on testing)
        self.axis_linear = 7  # Left stick up/down
        self.axis_angular = 6  # Left stick left/right
        self.scale_linear = 1.0
        self.scale_angular = 0.5

    def joy_callback(self, msg):
        # Read joystick values
        linear = msg.axes[self.axis_linear] * self.scale_linear  # Forward/backward
        angular = msg.axes[self.axis_angular] * self.scale_angular  # Left/right turn

        # Convert to motor duty cycles
        left_duty = linear + angular
        right_duty = linear - angular

        # Normalize values between -1 and 1
        left_duty = max(min(left_duty, 1.0), -1.0)
        right_duty = max(min(right_duty, 1.0), -1.0)

        print('left duty ', left_duty)
        print('right duty ', right_duty)

        # Publish motor commands
        #duty_msg = Float32MultiArray()
        #duty_msg.data = [left_duty, right_duty]
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg = DutyCycles() # Init
        msg.duty_cycle_left = left_duty
        msg.duty_cycle_right = right_duty
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = F710Teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()