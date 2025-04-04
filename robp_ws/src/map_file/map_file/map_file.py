#!/usr/bin/env python
import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class Map_file(Node):

    def __init__(self):
        super().__init__('map_file')

        # Declare parameters with default values
        self.box_threshold = self.get_parameter_or('box_threshold', 20)
        self.object_threshold = self.get_parameter_or('object_threshold', 5)
        self.msg_topic = self.get_parameter_or('msg_topic', '/classification/class')

        # Set QoS profile
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE

        # Subscribe to classification msg
        self.create_subscription(
            String, 
            self.msg_topic,
            self.map_callback, 
            qos_profile 
        )

        # Publisher for the objects
        self.publisher = self.create_publisher(MarkerArray, '/object_positions', 10)
        self.tf_pub = TransformBroadcaster(self)

        # BT communication
        self.trigger_sub = self.create_subscription(Bool, "/map_file/request", self.trigger_callback, 10)
        self.result_pub = self.create_publisher(Bool, "/map_file/result", 10)

        self.file = "map_file.txt"
        self.data = None
        self.map = []
        self.classifications = {'Box': 'B',
                                'Cube': '1',
                                'Sphere': '2',
                                'Animal': '3',}
        self.rev_classifications = {v: k for k, v in self.classifications.items()}


    def trigger_callback(self, msg: Bool):
        result_msg = Bool()
        result_msg.data = False

        if(msg.data):
            result_msg.data = self.perform_map_file_update()

        self.result_pub.publish(result_msg)

    def map_callback(self, msg: String):
        self.data = msg.data   


    def perform_map_file_update(self):
        data = self.data.split()
        classify, new_x, new_y, new_a = data[0], 100 * float(data[1]), 100 * float(data[2]), float(data[3]) % 180
        new_label = self.classifications.get(classify)
        new_votes = [0, 0, 0, 0] 
        classes = ['B', '1', '2', '3']

        for idx, (label, x, y, a, votes) in enumerate(self.map):
            distance = np.sqrt((x-new_x)**2+(y-new_y)**2)
            threshold = self.box_threshold if 'B' in (new_label, label) else self.object_threshold
            if distance < threshold:
                votes[classes.index(new_label)] += 1
                max_index = np.argmax(votes)
                if 'B' in (new_label, label):
                    avg_a = int(round((a + new_a) / 2, 0))
                else:
                    avg_a = 0
                self.map[idx] = (classes[max_index], 
                                 int(round((x + new_x) / 2, 0)),
                                 int(round((y + new_y) / 2, 0)),
                                 avg_a,
                                 votes)
                self.update_file()
                return True
            
        new_votes[classes.index(new_label)] += 1
        self.map.append((new_label, int(round(new_x, 0)), int(round(new_y, 0)), int(round(new_a, 0)), new_votes))
        self.update_file()

        return True

    def update_file(self):
        msg = MarkerArray()

        with open(self.file, 'w') as file:
            for label, x, y, a, _ in self.map:
                lenx = len(str(abs(x))) 
                leny = len(str(abs(y))) 
                # self.get_logger().info(f"lenx: {lenx}, x: {x}")
                space1 = 8 - lenx
                space2 = 8 - leny
                if x < 0:
                    space1 -= 1
                if y < 0:
                    space2 -= 1
                file.write(f"{label}       {x}{' '*space1}{y}{' '*space2}{a}\n")

                marker = Marker()
                marker.header.frame_id = label
                marker.pose.position.x = float(x)
                marker.pose.position.y = float(y)
                marker.pose.position.z = float(a)
                msg.markers.append(marker)
        
        self.publisher.publish(msg)
        self.broadcast_transforms()

    def broadcast_transforms(self):
        """Broadcasts transforms for each object in the map."""
        for idx, (label, x, y, a, _) in enumerate(self.map):
            
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = f"{self.rev_classifications.get(label)}_{idx}"

            # Set position
            transform.transform.translation.x = x / 100.0  # Convert back to meters
            transform.transform.translation.y = y / 100.0
            transform.transform.translation.z = 0.0  # Assuming it's on a 2D plane

            # Convert angle `a` to quaternion (for rotation around Z-axis)
            q = quaternion_from_euler(0, 0, np.deg2rad(a))
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]

            # Broadcast transform
            self.tf_pub.sendTransform(transform)


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
