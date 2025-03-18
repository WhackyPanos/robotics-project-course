#!/usr/bin/env python
import py_trees
from obstacle_on_path import CheckPath 

class ObstacleOnPath(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckPathInBT", node=None):
        super().__init__(name=name)
        self.node = node  
        self.check_path = CheckPath() 

    def setup(self, **kwargs):
        """ Setup function, called once before the first update. """
        self.check_path = kwargs['node']  

    def initialise(self):
        """ Called when the behavior starts (on the first tick). """
        self.get_logger().info("Starting to check path for obstacles.")

    def update(self):
        """ Behavior Tree update step. Called every tick of the BT. """
        if self.check_path.path is None or self.check_path.map is None:
            self.get_logger().info("Waiting for path or map data.")
            return py_trees.common.Status.RUNNING  

        clear_path = self.check_path.behaviour()
        self.get_logger().info("Checking if the path is clear of obstacles.")
        if clear_path:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        """ Called when the behavior finishes or is interrupted. """
        self.get_logger().info(f"Terminating CheckPathInBT with status {new_status}")
