#!/usr/bin/env python

import numpy as np
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid


class CheckPath(Node):

    def __init__(self):
        super().__init__('check_path')

        # subscribe to topics
        self.create_subscription(
            Path, 
            '/path_obs_avoid',  
            self.path_callback, 
            10)

        self.create_subscription(
            OccupancyGrid, 
            '/config_space',  
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
            i_x, i_y = self.world_to_grid(pose.pose.position.x, pose.pose.position.y) # Converts to gird indexes
            self.path.append((i_x, i_y)) 

    def map_callback(self, msg: OccupancyGrid):
        # Store latest map msg
        self.config_space = msg

    def behaviour(self):
        if self.path is None or self.config_space is None:
            self.get_logger().info("Waiting for path or map data.")
            return False

        path_len = len(self.path)
        for i in range(path_len - 1):
            start, end = self.path[i], self.path[i+1]
            if not self.raytrace(start, end):
                return False
        return True
    
    def raytrace(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        width = self.config_space.info.width
        height = self.config_space.info.height
        while True:
            if 0 <= x0 < width and 0 <= y0 < height:
                index = y0 * width + x0
                if self.config_space.data[index] >= 99: 
                    return False  
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return True 


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

