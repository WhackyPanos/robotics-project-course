#!/usr/bin/env python

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import tf_transformations

class path_planner(Node):
    def __init__(self):
        super().__init__('carrot_planner')

        self.goal = Point()
        self.goal.x = 0.0
        self.goal.y = 0.0
        self.is_goal = True
        
        self.create_subscription(PoseStamped, '/path', self.odometry_callback, 10)
        self.create_subscription(Point, '/goal', self.goal_callback, 10)
        self.goal_reached = self.create_publisher(Bool, '/goal_reached', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def odometry_callback(self, msg: PoseStamped):
        if self.is_goal == False:
            theta_g = math.atan2(self.goal.y - msg.position.y, 
                                 self.goal.x - msg.position.x)
            qz = msg.orientation.z

            [_,_,theta_r] = tf_transformations.euler_from_quaternion([0, 0, qz, 1])

            delta_theta = theta_g - theta_r
            if delta_theta >= 0.01:
                rotate = Twist()
                rotate.angular.z = 0.005 # TODO check angular vel
                self.publisher.publish(rotate)
            else:
                rotate = Twist()
                rotate.angular.z = 0.0
                self.publisher.publish(rotate)

                h = math.sqrt((self.goal.x - msg.position.x) ** 2 + 
                              (self.goal.y - msg.position.y) ** 2)
                if h >= 0.1:
                    move = Twist()
                    move.linear.x = 0.01
                    self.publisher.publish(move)
                else:
                    move = Twist()
                    move.linear.x = 0.0
                    self.publisher.publish(move)
                    self.goal_reached.publish(Bool(data=True))
                    self.is_goal = True

    def goal_callback(self, msg: Point):
        self.goal.x = msg.x
        self.goal.y = msg.y
        self.goal_reached.publish(Bool(data=False))
        self.is_goal = False
        return