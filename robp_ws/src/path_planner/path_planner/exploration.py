import rclpy
import pandas as pd
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid

class ExplorationNode(Node):
    def __init__(self):

        super().__init__('exploration')
        self.get_logger().info('Exploration has been started.')

        self.goal_reached = self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        self.map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        self.goal_publisher = self.create_publisher(PointStamped, '/temp_goal', 10)
    
    def goal_reached_callback(self, msg):
        if msg.data:
            self.timer = self.create_timer(2, self.publish_new_goal)

    def publish_new_goal(self, x, y):
        pub_msg = PointStamped()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.header.frame_id = "map"
        pub_msg.point.x = x
        pub_msg.point.y = y
        pub_msg.point.z = 0.0
        self.get_logger().info(f"Publishing new goal: x={x:.2f}, y={y:.2f}")
        self.publisher.publish(pub_msg)

        # Destroy the timer to ensure it only runs once
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None  # Reset the timer reference
    
    def map_callback(self, msg):
        pass
    
    def explore(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
