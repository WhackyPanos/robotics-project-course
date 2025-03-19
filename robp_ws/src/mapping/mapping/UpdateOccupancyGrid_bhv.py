#!/usr/bin/env python
import py_trees
import rclpy

# Note: this behiour only controls when the OccupancyGrid msg is published 

class UpdateOccupancyGrid(py_trees.behaviour.Behaviour):
    def __init__(self, name="UpdateOccupancyGridInBT", node=None):
        super().__init__(name=name)
        self.node = node
        self.occupancy_grid = None # Ros node
        #self.update_occupancy_grid.grid_update = False

    def setup(self, **kwargs):
        """ Setup function, called once before the first update. """
        self.occupancy_grid = kwargs['node']  
        rclpy.get_global_executor().add_node(self.occupancy_grid) 

    def initialise(self):
        """ Called when the behavior starts (on the first tick). """
        self.get_logger().info("Update occupancy grid behvior initialized")

    def update(self):
        """ Behavior Tree update step. Called every tick of the BT. """
        self.occupancy_grid.publish_current_grid()
        self.node.get_logger().info("New occupancy grid published")
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """ Called when the behavior finishes or is interrupted. """
        self.get_logger().info(f"Terminating UpdateOccupancyGridInBT with status {new_status}")
