#!/usr/bin/env python
import py_trees
import rclpy
from rclpy.node import Node
from .A_star import Planner_A_star

class PathPlan(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="PathPlannerBT", node=None):
        #super().__init__(name=name)
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)
        self.path_planner = Planner_A_star() # Ros node

    def setup(self, **kwargs):
        """ Setup function, called once before the first update. """
        #self.path_planner = kwargs['node']  
        #rclpy.get_global_executor().add_node(self.path_planner) 

    def initialise(self):
        """ Called when the behavior starts (on the first tick). """
        self.get_logger().info("Path planner behavior initialized")

    def update(self):
        """ Behavior Tree update step. Called every tick of the BT. """
        return py_trees.common.Status.SUCCESS if self.path_planner.path_plan(goal_threash=0.15) else py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        """ Called when the behavior finishes or is interrupted. """
        self.get_logger().info(f"Terminating PathPlannerBT")