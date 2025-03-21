import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from random import uniform
import numpy as np

class RandomPoint(Node):
    def __init__(self):
        super().__init__('random_point')
        
        self.map_subscription = self.create_subscription(OccupancyGrid, '/occupancy_grid', self.map_callback, 10)
        self.publisher = self.create_publisher(PointStamped, '/goal_point', 10)
        
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 1.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        
    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
 
    def is_not_occupied(self, x, y):
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            return self.map_data[grid_y, grid_x] != 100  
        return False

    def generate_new_point(self):
        if self.map_data is None:
            self.get_logger().warn("Map data not available yet!")
            return
        
        while True:
            x_desired = uniform(self.map_origin_x, self.map_origin_x + self.map_width * self.map_resolution)
            y_desired = uniform(self.map_origin_y, self.map_origin_y + self.map_height * self.map_resolution)
            if self.is_not_occupied(x_desired, y_desired):
                break

        pub_msg = PointStamped()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.header.frame_id = "map"
        pub_msg.point.x = x_desired
        pub_msg.point.y = y_desired
        pub_msg.point.z = 0.0
        self.get_logger().info(f"Publishing new goal: x={x_desired:.2f}, y={y_desired:.2f}")
        self.publisher.publish(pub_msg)


def main():
    rclpy.init()
    node = RandomPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
