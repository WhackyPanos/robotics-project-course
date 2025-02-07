#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class path_planner(Node):
    def __init__(self):
        super().__init__('carrot_planner')
        self.odom = self.create_subscription(Pose, '/odom', self.odometry_callback, 10)
        self.goal = self.create_subscription(Pose, '/goal', self.goal_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def odometry_callback(self):
        return

    def goal_callback(self):

        return
