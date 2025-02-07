#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from random import randrange as rr

# This node should generate a new random point (within a predefined rectangle) and publish it to the carrot_planner node
# (path_planner package). This should happen whenever it receives a message from the /goal/reached topic

# If the goal/reached topic has a true message, I publish a point
class point_generator(Node):
    def __init__(self):
        super().__init__('point_generator')
        self.subscription = self.create_subscription(Bool, '/goal/reached', self.goal_reached_callback, 10)
        self.publisher = self.create_publisher(Point, '/goal', 10)
        self.width = 1
        self.height = 1
        self.border_safety_margin = 0.1 # % of the width/height

    def goal_reached_callback(self, msg):
        if msg.data:
            x_desired = rr(0+self.width*self.border_safety_margin, self.width*(1-self.border_safety_margin))
            y_desired = rr(0+self.height*self.border_safety_margin, self.height*(1-self.border_safety_margin))

            pub_msg = Point()
            pub_msg.x = x_desired
            pub_msg.y = y_desired
            pub_msg.z = 0

            self.publisher.publish(pub_msg)
            

        return

