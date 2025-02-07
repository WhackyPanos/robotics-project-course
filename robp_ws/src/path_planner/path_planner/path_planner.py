#!/usr/bin/env python

import rclpy
import math
from rclpy.node import Node

def normalize_angle(angle):
    """Normalize angle to the range [-180, 180] degrees."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


class path_planner(Node):
    def __init__(self):
        super().__init__('carrot_planner')
        self.cur_pos = self.create_subscription(PoseStamped, '/path', self.odometry_callback, 10)
        self.goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.goal_reached = self.create_publisher(Bool, '/goal_reached', 10) #this should be a boolean
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def odometry_callback(self):
        # check if we reached the goal position and if so, we publish /goal/reached message
        return

    def goal_callback(self):
        # run once we received a new goal

        # first rotate robot to face the correct position
        theta_g = math.degrees(math.atan2(self.goal.position.y - self.cur_pos.position.y, self.goal.position.x - self.cur_pos.position.x))
        delta_theta = normalize_angle(theta_g - self.cur_pos.orientation.z)

        # then give a linear vel command to move the robot

        # when the robot reaches its goal, the odometry_callback will detect it and stop it.
        return