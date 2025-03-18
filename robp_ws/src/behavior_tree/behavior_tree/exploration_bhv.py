#!/usr/bin/env python
from std_msgs.msg import Bool
from rclpy.node import Node
import py_trees
import rclpy
from robp_interfaces.msg import BoxPosition, ObjectPosition, Path

from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from handle_objects.ik_solver import IKNode
import PyKDL as kdl

from robp_interfaces.srv import BoxPositionSrv, ObjectPositionSrv
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from math import pi, acos, atan2, atan, cos, sin, sqrt
import numpy as np 



class UnexploredMap(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="PlanPath"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

class PathPlan(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="PlanPath"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""

    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

class NavigateToGoal(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="PlanPath"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""

    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

class UpdateMap(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="PlanPath"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""

    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

class ObstacleOnPath(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="PlanPath"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""

    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

class Classify(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="PlanPath"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""

    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

# ----------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

class NewObject(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="PlanPath"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""

    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
