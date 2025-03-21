#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from geometry_msgs.msg import Pose2D, PointStamped
import numpy as np
from math import sqrt


class I_NextObject(py_trees.behaviour.Behaviour, Node):
    def __init__(self, obj_list, name):
        super().__init__(name=name)
        self.obj_list = obj_list
        self.get_robot_pos = np.array([0,0,0])

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs["node"]
        self.robot_pos_sub = self.node.create_subscription(Pose2D, '/odom_pose', self.get_robot_pos, 10)
        self.next_goal_pub = self.node.create_publisher(PointStamped,'/temp_goal', 10 )


    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""


    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        # compute closest object
        distances = [sqrt((self.get_robot_pos[0]-self.obj_list[1])**2 + (self.get_robot_pos[0]-self.obj_list[1])**2 )]
        

        # publish goal point (object to pick)


    def get_robot_pos(self, msg):
        self.get_robot_pos[0] = msg.x
        self.get_robot_pos[1] = msg.y
        self.get_robot_pos[2] = msg.theta
