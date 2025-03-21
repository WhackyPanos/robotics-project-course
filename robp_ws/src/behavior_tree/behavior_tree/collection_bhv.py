#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from geometry_msgs.msg import Pose2D, PointStamped
from std_msgs.msg import Bool, String
import numpy as np
from math import sqrt
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException



class UpdateObjectList(py_trees.behaviour.Behaviour, Node):
    def __init__(self, obj_list, box_list ,name ='UpdateObjectList'):
        super().__init__(name=name)
        self.obj_list = obj_list
        self.box_list = box_list
        self.robot_pos = np.array([0,0])

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs["node"]
        # TODO: transform to map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.to_frame_rel = 'map'
        self.from_frame_rel = 'odom'

        self.robot_pos_sub = self.node.create_subscription(Pose2D, '/odom_pose', self.get_robot_pos_callback, 10)
        self.need_next_object_sub = self.node.create_subscription(String, '/next_goal/object/need', self.need_next_object_callback, 10)
        self.update_object_list_sub = self.node.create_subscription(PointStamped, '/next_goal/object/update', self.update_object_list_callback, 10)

        self.next_goal_pub = self.node.create_publisher(PointStamped,'/motion/goal', 10 )
        self.need_next_object = 'Object' # at the beginning, we want to pick objects

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        if self.need_next_object is not None:
            if self.need_next_object == 'Object':
                points_list = self.obj_list
            else:
                points_list = self.box_list
            # compute closest object
            distances = np.array([sqrt((self.robot_pos[0]-obj[1])**2 + (self.robot_pos[1]-obj[2])**2) for obj in points_list])
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
        """ Get robot position in odom frame and convert to map frame. If transform fails, use odom frame since
        we just need a rough estimate of the closest object """
    
        try:
            t = self.tf_buffer.lookup_transform(
                self.to_frame_rel,
                self.from_frame_rel,
                rclpy.time.Time(seconds=0))
            stamp = t.header.stamp
            
            point_odom = PointStamped()
            point_odom.header.frame_id = 'odom'
            point_odom.header.stamp = stamp
            point_odom.point.x = msg.x
            point_odom.point.y = msg.y
            point_odom.point.z = 0.0

            point_map = self.tf_buffer.transform(point_odom, self.to_frame_rel)
            self.robot_pos[0] = point_map.point.x
            self.robot_pos[1] = point_map.point.y

        except TransformException as ex:
            self.robot_pos[0] = msg.x
            self.robot_pos[1] = msg.y


        return

    def need_next_object_callback(self, msg):
        self.need_next_object = msg.data

    def update_object_list_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.obj_list = [obj for obj in self.obj_list if obj[1] != x and obj[2] != y]


class ArmTaskSucceeded(py_trees.behaviour.Behaviour, Node):
    def _init__(self, name = 'ArmTaskSucceeded'):
        super().__init__(name=name)
        self.arm_task = 'pick' # can be either 'pick' or 'place'
    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.picklift_sub = self.node.create_subscription(Bool, '/picklift/succeded', self.picklift_callback, 10)
        self.next_goal = self.node.create_publisher(String, '/next_goal/object/need', 10)
        self.msg = String()

    def update(self):
        # Note: hope the behavior tick is slower than the subscriber callback, might need changes
        self.next_goal.publish(self.msg)
        return py_trees.common.Status.FAILURE

    def picklift_callback(self, msg):
        if not msg.data: # pick_lift failed, remove object from list and try same movement (pick object or place on box)
            self.msg.data = 'Object' if self.arm_task == 'pick' else 'Box'
            pass
        else: # pick lift succeeded, remove object from list and try placing now
            self.msg.data = 'Box' if self.arm_task == 'pick' else 'Object'
            self.arm_task = 'place' if self.msg.data == 'Box' else 'pick'
            pass
        return
        

