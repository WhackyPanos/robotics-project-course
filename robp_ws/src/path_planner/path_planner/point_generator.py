#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PointStamped
from random import uniform as u

# This node should generate a new random point (within a predefined rectangle) and publish it to the carrot_planner node
# (path_planner package). This should happen whenever it receives a message from the /goal/reached topic

# If the goal/reached topic has a true message, I publish a point
class RandomPointGenerator(Node):
    def __init__(self):
        super().__init__('point_generator')
        self.subscription = self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        self.publisher = self.create_publisher(PointStamped, '/temp_goal', 10)

        # Declare parameters with default values. If the parameters are set in the launch file, the default values will be overwritten
        # check config folder for params.yaml
        self.declare_parameter('width', 1.0)
        self.declare_parameter('height', 1.0)
        self.declare_parameter('border_safety_margin', 0.1)

        # Retrieve parameter values
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.border_safety_margin = self.get_parameter('border_safety_margin').value

        self.timer = None  # Initialize the timer as None
        self.get_logger().info(f"Point Generator initialized with width={self.width}, height={self.height}, margin={self.border_safety_margin}")

    def goal_reached_callback(self, msg):
        if msg.data:
            self.timer = self.create_timer(2, self.publish_new_goal)
    
    def publish_new_goal(self):
        x_desired = u(0+self.width*self.border_safety_margin, self.width*(1-self.border_safety_margin))
        y_desired = u(0+self.height*self.border_safety_margin, self.height*(1-self.border_safety_margin))

        pub_msg = PointStamped()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.header.frame_id = "map"
        pub_msg.point.x = x_desired
        pub_msg.point.y = y_desired
        pub_msg.point.z = 0.0
        self.get_logger().info(f"Publishing new goal: x={x_desired:.2f}, y={y_desired:.2f}")
        self.publisher.publish(pub_msg)

         # Destroy the timer to ensure it only runs once
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None  # Reset the timer reference
    
def main(args=None):
    rclpy.init(args=args)
    node = RandomPointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

