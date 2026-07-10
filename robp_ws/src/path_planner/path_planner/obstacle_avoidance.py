#!/usr/bin/env python

import rclpy
import math
import time
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.create_timer(0.1, self.timer_callback)


    def lidar_callback(self, msg):
        # Do something with the lidar data
        pass

    def checkForObstacle(self, lidar):
        # Check if there is an obstacle in front of the robot
        pass

    