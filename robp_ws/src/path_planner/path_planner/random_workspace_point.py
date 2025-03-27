import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped
from random import uniform

class RandomPoint(Node):
    def __init__(self):
        super().__init__('random_point')
        
        # Subscriptions
        self.space_subscription = self.create_subscription(OccupancyGrid, '/config_space', self.space_callback, 10)
        self.grid_subscription = self.create_subscription(OccupancyGrid, '/occupancy_grid', self.grid_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(PoseStamped, '/motion/goal', 10)
        
        # Map attributes
        self.map_data = None
        self.space_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 1.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

    def grid_callback(self, msg):
        """ Stores occupancy grid data (-1 unexplored, 0 explored, 99 object, 100 occupied). """
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def space_callback(self, msg):
        """ Stores configuration space (1 = occupied, 0 = free). """
        self.space_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def is_valid_point(self, x, y):
        """ Checks if a point is free in config_space (1) and unexplored in occupancy_grid (-1). """
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)

        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            # Ensure point is in free space (config_space == 0) and not explored (occupancy_grid == -1)
            return self.space_data[grid_y, grid_x] == 0 and self.map_data[grid_y, grid_x] == -1
        return False

    def generate_new_point(self):
        """ Generates a random valid point in unexplored free space and publishes it. """
        if self.map_data is None or self.space_data is None:
            self.get_logger().warn("Map or config space data not available yet!")
            return False

        for _ in range(1000):  # Avoid infinite loops by limiting attempts
            x_desired = uniform(self.map_origin_x, self.map_origin_x + self.map_width * self.map_resolution)
            y_desired = uniform(self.map_origin_y, self.map_origin_y + self.map_height * self.map_resolution)
            
            if self.is_valid_point(x_desired, y_desired):
                pub_msg = PoseStamped()
                pub_msg.header.stamp = self.get_clock().now().to_msg()
                pub_msg.header.frame_id = "map"
                pub_msg.pose.position.x = x_desired
                pub_msg.pose.position.y = y_desired
                pub_msg.pose.position.z = 0.0
                pub_msg.pose.orientation.x = 0.0
                pub_msg.pose.orientation.y = 0.0
                pub_msg.pose.orientation.z = 0.0
                pub_msg.pose.orientation.w = 1.0
                self.get_logger().info(f"Publishing new goal: x={x_desired:.2f}, y={y_desired:.2f}")
                self.publisher.publish(pub_msg)
                return True

        self.get_logger().warn("Could not find a valid random point!")
        return False


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
