#!/usr/bin/env python
import py_trees
import rclpy
from rclpy.node import Node
from .obstacle_on_path import CheckPath 

class ObstacleOnPath(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="CheckPathInBT"):
        # Initialize Behaviour (PyTrees) and Node (ROS2)
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node

        self.check_path = CheckPath() 

    def setup(self, **kwargs):
        """ Setup function, called once before the first update. """  

    def initialise(self):
        """ Called when the behavior starts (on the first tick). """
        self.get_logger().info("Starting to check path for obstacles.")

    def update(self):
        """ Behavior Tree update step. Called every tick of the BT. """
        clear_path = self.check_path.behaviour()
        if clear_path:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status):
        """ Called when the behavior finishes or is interrupted. """
        
