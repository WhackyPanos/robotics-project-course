#!/usr/bin/env python

import numpy as np
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from scipy.ndimage import binary_dilation


class CheckPath(Node):

    def __init__(self):
        super().__init__('check_path')

        # subscribe to topics
        self.create_subscription(
            Path, 
            '/full_path',  # Change to path topic name
            self.path_callback, 
            10)

        self.create_subscription(
            OccupancyGrid, 
            '/occupancy_grid',  
            self.map_callback, 
            10)

        self.path = None
        self.map = None
        self.robot_radius = 0.2 # May need to be adjusted

    def path_callback(self, msg: Path):
        # Store latest path msg
        self.path = []
        for pose in msg.poses:
            self.path.append((pose.pose.position.x, pose.pose.position.y))

    def map_callback(self, msg: OccupancyGrid):
        # Store latest map msg
        self.map = msg

    def behaviour(self):
        if self.path is None or self.map is None:
            self.get_logger().info("Waiting for path or map data.")
            return False
                
        width = map.info.width
        height = map.info.height
        map_data = np.array(map.data).reshape((height, width))  

        inflated_map = self.inflate_map(map_data)

        for index in self.path:
            x, y = index
            if inflated_map[x, y] == 1:
                return False
    
    def inflate_map(self, grid): # Create the configurations space
        binary_grid = np.zeros_like(grid)
        binary_grid[grid == 100] = 1
        
        # Calculate kernel size based on robot radius and map resolution
        kernel_radius = int(np.ceil(self.robot_radius / self.map_info.resolution))
        
        # Create circular kernel for dilation
        y, x = np.ogrid[-kernel_radius:kernel_radius+1, -kernel_radius:kernel_radius+1]
        kernel = x**2 + y**2 <= kernel_radius**2
        
        # Dilate obstacles to create configuration space
        config_space = binary_dilation(binary_grid, kernel).astype(np.int8)        
        
        return config_space


# def main():
#     rclpy.init()
#     node = CheckPath()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

