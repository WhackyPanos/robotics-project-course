#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from geometry_msgs.msg import Pose2D, PointStamped
from std_msgs.msg import Bool
import numpy as np
from math import sqrt


class I_ObjectList(py_trees.behaviour.Behaviour, Node):
    def __init__(self, obj_list, name):
        super().__init__(name=name)
        self.obj_list = obj_list
        self.get_robot_pos = np.array([0,0,0])

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs["node"]
        self.robot_pos_sub = self.node.create_subscription(Pose2D, '/odom_pose', self.get_robot_pos_callback, 10)
        self.next_goal_pub = self.node.create_publisher(PointStamped,'/temp_goal', 10 )
        self.need_next_object_sub = self.node.create_subscription(Bool, '/next_goal/object/need', self.need_next_object_callback, 10)
        self.update_object_list_sub = self.node.create_subscription(PointStamped, '/next_goal/object/update', self.update_object_list_callback, 10)
        self.need_next_object = False


    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""


    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        if self.need_next_object:
            # compute closest object
            distances = np.array([sqrt((self.get_robot_pos[0]-obj[1])**2 + (self.get_robot_pos[0]-obj[1])**2) for obj in self.obj_list])
            closest_obj = self.obj_list[np.argmin(distances)]

            # publish goal point (object to pick)
            msg = PointStamped()
            msg.point.x = closest_obj[1]
            msg.point.y = closest_obj[2]
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            self.next_goal_pub.publish(msg)
        return py_trees.common.Status.SUCCESS


    def get_robot_pos_callback(self, msg):
        self.get_robot_pos[0] = msg.x
        self.get_robot_pos[1] = msg.y
        self.get_robot_pos[2] = msg.theta

    def need_next_object_callback(self, msg):
        self.need_next_object = msg.data

    def update_object_list_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.obj_list = [obj for obj in self.obj_list if obj[1] != x and obj[2] != y]
