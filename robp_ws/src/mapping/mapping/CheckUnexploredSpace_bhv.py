#!/usr/bin/env python
import py_trees
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CheckOccupancyGrid(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="CheckOccupancyGridBT"):
        # Initialize Behaviour (PyTrees) and Node (ROS2)
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node

        self.map_explored = False

    def setup(self, **kwargs):
        """ Setup function, called once before the first update. """
        self.node = kwargs['node']

        # Subscribe to clustering result
        self.result_sub = self.node.create_subscription(
            OccupancyGrid, "/occupancy_grid", self.map_callback, 10)

        
    def initialise(self):
        """ Called when the behavior starts (on the first tick). """
        self.map_explored = False

    def update(self):
        """ Behavior Tree update step. Called every tick of the BT. """
        if self.map_explored:
            return py_trees.common.Status.SUCCESS
        else: return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status: py_trees.common.Status):
        """ Called when the behavior finishes or is interrupted. """

    def map_callback(self, msg:OccupancyGrid, threshold = 5):
        # Get the data from the occupancy grid message
        grid_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)

        # Count the number of unexplored cells (-1)
        unexplored_cells = np.sum(grid_data == -1)

        # Total number of cells in the grid
        total_cells = grid_data.size

        # Calculate the percentage of unexplored cells
        unexplored_percentage = (unexplored_cells / total_cells) * 100

        self.get_logger().info(f"Percentage of unexplored space:: {unexplored_percentage}")

        # Check if the percentage of unexplored cells is above the threshold
        self.map_explored = unexplored_percentage < threshold