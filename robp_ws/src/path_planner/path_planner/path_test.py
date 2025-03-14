import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import random

"""
This is just a test for the Path functionality of the motion control node.
One can provide it a goal and it generates and publishes a curvy path to that goal with x increment.
"""


class PathPublisher(Node):


    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, '/motion/path', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_path)
        self.get_logger().info('Path Publisher Node has been started.')

    def publish_path(self):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Create a high resolution curvy path with 0.2 increments
        goal_x = 5.0  # Example goal x-coordinate
        goal_y = 5.0  # Example goal y-coordinate
        step = 0.2
        x = 0.0
        y = 0.0

        while x <= goal_x and y <= goal_y:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
            x += step
            y += step * (goal_y / goal_x)  # Adjust y to create a curvy path
            
            # Add noise to the points
            x += random.uniform(-0.1, 0.1)
            y += random.uniform(-0.1, 0.1)

        self.publisher_.publish(path_msg)
        self.get_logger().info('Publishing path message.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()