#!/usr/bin/env python
# stuff
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Header
from robp_interfaces.msg import DutyCycles
from geometry_msgs.msg import Twist

class Twist2Duty(Node):
    def __init__(self):
        super().__init__('twist2duty')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.twist2duty_callback, 10)
        self.publisher = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)

        # Define joystick mappings (adjust based on testing)    
        self.axis_straight = 7  # Left stick up/down
        self.axis_rotate = 2  # CHANGE LATER ON
        #self.axis_angular = 6  # Left stick left/right
        self.max_linear = 0.1 # maximum linear velocity
        self.max_angular = 0.05 # maximum angular velocity

        # Robot parameters
        self.wheel_radius = 0.04921  # meters
        self.wheel_base = 0.3125  # meters

    def twist2duty_callback(self, msg):
        # Convert from cmd_vel to motor duty cycles
        v = msg.linear.x
        w = msg.angular.z 

        # if we are using the controller
        right_duty = v/self.max_linear + w/self.max_angular
        left_duty = v/self.max_linear - w/self.max_angular

        print('left duty ', left_duty)
        print('right duty ', right_duty)

        # Publish motor commands
        duty_msg = DutyCycles() # Init
        duty_msg.header = Header()
        duty_msg.header.stamp = self.get_clock().now().to_msg()

        duty_msg.duty_cycle_left = left_duty
        duty_msg.duty_cycle_right = right_duty
        self.publisher.publish(duty_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Twist2Duty()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()