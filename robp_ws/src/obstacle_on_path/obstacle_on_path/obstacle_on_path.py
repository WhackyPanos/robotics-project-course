#!/usr/bin/env python

import numpy as np
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
# Import inflate map

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
        
        map = self.map
        # map = import.inflate_map(self.map)
        
        width = map.info.width
        height = map.info.height
        map_data = np.array(map.data).reshape((height, width))  

        for index in self.path:
            x, y = index
            if map_data[x, y] == 100:
                return False
        
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

