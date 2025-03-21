#!/usr/bin/env python

import numpy as np
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
import binary


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
        self.config_space = None

    
    def world_to_grid(self, x, y):
        '''Converts world coordinates in [m] to grid indices.'''
        resolution = self.config_space.info.resolution
        origin_x = self.config_space.info.origin.position.x
        origin_y = self.config_space.info.origin.position.y

        i_x = int((x - origin_x) / resolution)    
        i_y = int((y - origin_y) / resolution)
        
        return i_x, i_y

    
    
    def path_callback(self, msg: Path):
        # Store latest path msg
        self.path = []
        for pose in msg.poses:
            self.path.append((pose.pose.position.x, pose.pose.position.y))

    def map_callback(self, msg: OccupancyGrid):
        # Store latest map msg
        self.map = msg

    def behaviour(self):
        if self.path is None or self.config_space is None:
            self.get_logger().info("Waiting for path or map data.")
            return False
        
        map = self.inflate_map(self.map)
        
        width = map.info.width
        height = map.info.height
        map_data = np.array(map.data).reshape((height, width))  

        for index in self.path:
            x, y = index
            if map_data[x, y] == 100:
                return False
    
    def inflate_map(self, occupancy_grid_msg): # Create the configurations space
        # The problem with doing path palnning when there is a new map topic is 
        # Convert occupancy grid to numpy array
        self.map_info = occupancy_grid_msg.info # Gets info from the occupancy grid

        # Create binary occupancy grid (1 for obstacles, 0 for free space)
        grid = np.array(occupancy_grid_msg.data).reshape(self.map_info.height, self.map_info.width)
        binary_grid = np.zeros_like(grid)
        
        # Threshold for obstacles (usually >50 is considered occupied)
        binary_grid[grid > 50] = 1
        # Robot can drive through both known and unknown space
        
        # Calculate kernel size based on robot radius and map resolution
        kernel_radius = int(np.ceil(self.robot_radius / self.map_info.resolution))
        
        # Create circular kernel for dilation
        y, x = np.ogrid[-kernel_radius:kernel_radius+1, -kernel_radius:kernel_radius+1]
        kernel = x**2 + y**2 <= kernel_radius**2
        
        # Dilate obstacles to create configuration space
        self.config_space = binary_dilation(binary_grid, kernel).astype(np.int8)
        self.get_logger().info(f'Configuration space created with robot radius: {self.robot_radius}m')
        
        return



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

