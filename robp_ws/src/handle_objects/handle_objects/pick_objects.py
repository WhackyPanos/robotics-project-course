#!/usr/bin/env python

import py_trees
import py_trees_ros
import rclpy
import time
#from rclpy.action import ActionClient
#from example_interfaces.action import Move  # Replace with your actual action
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension




class ObjTuckArm(py_trees.behaviour.Behaviour): # this class is a py_tree node and a ros node
    def __init__(self, name="ObjTuckArm"):
        super().__init__(name=name)
        #self.logger.debug("ObjTuckArm was called.")
        #self.cached_context = None
    
    def setup(self, **kwargs):
            """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
            print("Setting up ObjTuckArm node.")
            self.node = kwargs['node']
            self.ota_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = 10) # ota = object_tuck_arm
            self.init_tuck_arm_time = 2000 # in ms
            self.arm_moving = True

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""
        print("Initializing ObjTuckArm behavior.")
        self.arm_moving = False
        #self.timer = self.node.create_timer(self.init_tuck_arm_time*2, self.finish_tuck_arm)

    def finish_tuck_arm(self):
        """ Callback to mark arm movement as complete after delay. """
        print("Arm tucking finished.")
        self.arm_moving = False
        self.timer.cancel()  # Stop the timer
         
         
    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        print(f"Ticking ObjTuckArm, it is {self.arm_moving} that the arm is moving")
        if self.arm_moving:
            return py_trees.common.Status.RUNNING  # Keep running while the arm moves

        # Publish the tuck command
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
            data_offset=0
        )      
        angles = [10000] * 6
        times = [self.init_tuck_arm_time] * 6
        msg.data = angles + times
        print(f"About to publish message")
        self.ota_publisher_.publish(msg)
        print("Publishing tuck arm command.")

        return py_trees.common.Status.SUCCESS







 # -----------------------------------------------------------------------------------------------------------------------

class Pick(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Pick"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, "pick_node")  # ROS 2 node initialization


    def setup(self):
         """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""

    def initialise(self):
         """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""
         
    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """


class InitTuckArm(py_trees.behaviour.Behaviour): # this class is a py_tree node and a ros node
    def __init__(self, name="InitTuckArm"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, "init_tuck_arm_node")  # ROS 2 node initialization
        self.get_logger().info("InitTuckArm node was called.")
        self.init_tuck_arm_time = 1000 # in ms
        self.init_tuck_arm_angles = 12000 # in centi-degrees
        self.ita_publisher_ = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10) # ita = init_tuck_arm

        self.arm_moving = False
        self.timer = None

    def setup(self):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        
        self.get_logger().info("Setting up InitTuckArm node.")

    def initialise(self):
        """ Called before execution. Resets state. """
        self.get_logger().info("Initializing InitTuckArm behavior.")
        self.arm_moving = True

        # Publish the tuck command
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
            data_offset=0
        )      
        angles = [self.init_tuck_arm_angles] * 6
        times = [self.init_tuck_arm_time] * 6
        msg.data = angles + times
        self.ita_publisher_.publish(msg)
        self.timer = self.create_timer(self.init_tuck_arm_time*2, self.finish_tuck_arm)
        self.get_logger().info("Publishing tuck arm command.")

    def finish_tuck_arm(self):
        """ Callback to mark arm movement as complete after delay. """
        self.get_logger().info("Arm tucking finished.")
        self.arm_moving = False
        self.timer.cancel()  # Stop the timer

         
    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        if self.arm_moving:
            return py_trees.common.Status.RUNNING  # Keep running while the arm moves

        return py_trees.common.Status.SUCCESS