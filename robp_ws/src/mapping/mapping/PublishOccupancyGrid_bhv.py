#!/usr/bin/env python
import py_trees
import rclpy
from rclpy.node import Node
from .occupancy_grid import OccupancyGridNode

# Note: this behiour only controls when the OccupancyGrid msg is published 

class PublishOccupancyGrid(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="PublishOccupancyGridBT"):
        # Initialize Behaviour (PyTrees) and Node (ROS2)
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node

        self.occupancy_grid =  OccupancyGridNode() # Ros node

    def setup(self, **kwargs):
        """ Setup function, called once before the first update. """
        # self.occupancy_grid = kwargs['node']  
        #rclpy.get_global_executor().add_node(self.occupancy_grid) 

    def initialise(self):
        """ Called when the behavior starts (on the first tick). """
        pass
        #self.get_logger().info("Publish occupancy grid behavior initialized")

    def update(self):
        """ Behavior Tree update step. Called every tick of the BT. """
        self.occupancy_grid.publish_current_grid()
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status: py_trees.common.Status):
        """ Called when the behavior finishes or is interrupted. """
        pass
        #self.get_logger().info(f"Terminating PublishOccupancyGridBT")
