#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import py_trees
from path_planner.path_planner import CarrotPlanner
from path_planner.point_generator import RandomPointGenerator
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped


class goCollect(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="goCollect"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.point_generator = RandomPointGenerator() #trigger to start publishing points
        self.planner = CarrotPlanner()
        self.node = kwargs["node"]
        self.point_reached = False
        self.moving = False
        self.node.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        self.node.create_subscription(PointStamped, '/temp_goal', self.point_published_callback, 10)
        
    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""
        pass
        

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        if self.point_reached:
            return py_trees.common.Status.SUCCESS
        elif self.moving and not self.point_reached:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING



    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

    def point_published_callback(self, msg):
        # robot will start moving
        if msg is not None:
            self.moving = True
            self.node.get_logger().info("Robot is moving to pick location!")
            self.timer.cancel()

    def goal_reached_callback(self, msg):
        if self.moving and msg.data: # robot is moving and has reached the goal point
            self.point_reached = True