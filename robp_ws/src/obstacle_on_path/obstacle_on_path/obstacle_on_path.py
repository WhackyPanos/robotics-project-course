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
            '/path_topic_name',  # Change to path topic name
            self.path_callback, 
            10
        )

        self.create_subscription(
            OccupancyGrid, 
            '/map',  
            self.map_callback, 
            10
        )

        self.path = None
        self.map = None

    def path_callback(self, msg: Path):
        # Store latest path msg
        self.path = msg

    def map_callback(self, msg: OccupancyGrid):
        # Store latest map msg
        self.map = msg

    def raytrace(self, start, end, width, height):
        (x0, y0) = start
        (x1, y1) = end
        (dx, dy) = (math.fabs(x1 - x0), math.fabs(y1 - y0))
        sx = 1 if x0 < x1 else -1 # Different types of slopes described by sx and sy
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        traversed = []
        while x0 != x1 or y0 != y1:
            if 0 <= x0 < self.width and 0 <= y0 < self.height:
                traversed.append((x0, y0))
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy        
        return traversed # returns a lsit of cell indexes from start to one cell before end

    def behaviour(self):
        if self.path is None or self.map is None:
            self.get_logger().info("Waiting for path or map data.")
            return 
        
        map = self.map
        
        origin_x = map.info.origin.position.x
        origin_y = map.info.origin.position.y
        resolution = map.info.resolution  
        width = map.info.width
        height = map.info.height
        map_data = np.array(map.data).reshape((height, width))  

        for i in range(len(self.path.poses) - 1):
            p1 = self.path.poses[i].pose.position
            p2 = self.path.poses[i + 1].pose.position

            grid_x0 = int((p1.x - origin_x) / resolution)
            grid_y0 = int((p1.y - origin_y) / resolution)
            grid_x1 = int((p2.x - origin_x) / resolution)
            grid_y1 = int((p2.y - origin_y) / resolution)

            line_cells = self.raytrace(grid_x0, grid_y0, grid_x1, grid_y1, width, height)

            for gx, gy in line_cells:
                if 0 <= gx < width and 0 <= gy < height:
                    if map_data[gy, gx] == 100:
                        self.get_logger().info(f"Obstacle detected between {p1.x},{p1.y} and {p2.x},{p2.y}")
                        return False  

        self.get_logger().info("Path is clear.")
        return True

def main():
    rclpy.init()
    node = CheckPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()

