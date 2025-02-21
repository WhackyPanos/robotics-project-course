#!/usr/bin/env python
import os
import math
import string
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import String

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_geometry_msgs


class Map_file(Node):

    def __init__(self):
        super().__init__('map_file')

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            String, '/detection/class', self.map_callback, 10)

        self.file = "map_file.txt"
        self.map = []
        self.classifications = {'Box': 'B',
                                'Cube': '1',
                                'Sphere': '2',
                                'Animal': '3',}

    def map_callback(self, msg: String):
        data = msg.data.split()
        classify, new_x, new_y, new_a = data[0], float(data[1]), float(data[2]), float(data[3])
        new_label = self.classifications.get(classify, '3')
        new_votes = [0, 0, 0, 0] 
        classes = ['B', '1', '2', '3']

        for idx, (label, x, y, a, votes) in enumerate(self.map):
            distance = np.sqrt((x-new_x)**2+(y-new_y)**2)
            threshold = 0.2 if 'B' in (new_label, label) else 0.1
            if distance < threshold:
                votes[classes.index(new_label)] += 1
                max_index = np.argmax(votes)
                self.map[idx] = (classes[max_index], 
                                 round((x + new_x) / 2, 2),
                                 round((y + new_y) / 2, 2),
                                 round((a + new_a) / 2, 0),
                                 votes)
                self.update_file()
                return
            
        new_votes[classes.index(new_label)] += 1
        self.map.append((new_label, new_x, new_y, new_a, new_votes))
        self.update_file()

    def update_file(self):
        with open(self.file, 'w') as file:
            for label, x, y, a, _ in self.map:
                file.write(f"{label} {x:.2f} {y:.2f} {a:.0f}\n")


def main():
    if os.path.exists("map_file.txt"):
        os.remove("map_file.txt")
    rclpy.init()
    node = Map_file()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
