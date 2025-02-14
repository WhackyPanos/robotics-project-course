#!/usr/bin/env python
import rclpy
import math
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped, TransformStamped, PointStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D


class PickOrSearch(Node):
    def __init__(self):
        super().__init__('pick_or_search')



def main(args=None):
    rclpy.init(args=args)
    node = PickOrSearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()