#!/usr/bin/env python
import os
import math
import string
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_geometry_msgs

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from pyclustering.cluster.dbscan import dbscan

import ctypes
import struct

class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Define QoS profile: Keep only the last message
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Buffer size of 1 (latest message only)
            reliability=ReliabilityPolicy.BEST_EFFORT  # Best effort for sensor data
        )
        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, qos_profile)

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/camera/depth/color/ds_points', 10)

        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        self.file = "map_file.txt"
       

    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """

        stamp = msg.header.stamp
        frame = msg.header.frame_id

        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors = colors.astype(np.float32) / 255      


        #Passthrough filter
        prefilter_mask = (points[:, 2] < 1.0) & (points[:,1] < 0.08) & (points[:,1] > 0)
        prefilter_points = points[prefilter_mask]
        prefilter_points = prefilter_points[::20]
        # prefilter_colors = colors[prefilter_mask]

        self._logger.info(f"Prefilter points: {prefilter_points.shape}")

        #Perform DBSCAN clustering using PyClustering
        epsilon = 0.03  # Distance threshold
        min_samples = 10  # Minimum points to form a cluster

        prefilter_points_list = prefilter_points.tolist()
        if len(prefilter_points_list) >= min_samples:
            # Perform DBSCAN clustering
            db = dbscan(prefilter_points_list, epsilon, min_samples)
            db.process()

            # Get the resulting clusters
            clusters = db.get_clusters()
            self._logger.info(f"Clusters: {len(clusters)}")

            #For each cluster
            for cluster_idx, cluster in enumerate(clusters):
                self._logger.info(f"Processing cluster {cluster_idx} with {len(cluster)} points.")
                cluster_array = np.array(cluster)
                cluster_points = prefilter_points[cluster_array]
                # cluster_colors = prefilter_colors[cluster_array]

                box_mask = cluster_points[:, 1] < 0.03
                box_points = cluster_points[box_mask]
                # box_colors = cluster_colors[box_mask]
                box_detected = False

                if np.size(box_points[:,0]) > min_samples:
                    self.get_logger().info("Grey box detected!")
                    box_detected = True
                    box_centre = np.mean(box_points, axis=0)           
                    box_point_stamped = PointStamped()
                    box_point_stamped.header.stamp = stamp
                    box_point_stamped.header.frame_id = frame
                    box_point_stamped.point.x, box_point_stamped.point.y, box_point_stamped.point.z = box_centre

                    self.broadcast_tf_map(stamp, frame, box_point_stamped, 'box')

                
                object_mask = cluster_points[:,1] > 0.03
                object_points = cluster_points[object_mask]
                # object_colors = cluster_colors[object_mask]

                if(np.size(object_points[:,0])) > min_samples and not box_detected:
                    self.get_logger().info("Object detected!")

                    object_centre = np.mean(object_points, axis=0)           
                    object_point_stamped = PointStamped()
                    object_point_stamped.header.stamp = stamp
                    object_point_stamped.header.frame_id = frame
                    object_point_stamped.point.x, object_point_stamped.point.y, object_point_stamped.point.z = object_centre

                    self.broadcast_tf_map(stamp, frame, object_point_stamped, 'object')

                # # Detect objects based on color
                # red_mask = (box_colors[:, 0] > 0.8) & (box_colors[:, 1] < 0.4) 
                # green_mask = (box_colors[:, 0] < 0.1) & (box_colors[:, 1] > 0.4)
                # gray_mask = (box_colors[:,0] > 0.3) & (box_colors[:,0] < 0.7) & \
                #             (box_colors[:,1] > 0.3) & (box_colors[:,1] < 0.7) & \
                #             (box_colors[:,2] > 0.3) & (box_colors[:,2] < 0.7)

                # fields = [
                #     PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                #     PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                #     PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                #     PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
                # ]

                # points_with_rgb = [(x, y, z, self.pack_rgb(r, g, b)) for (x, y, z), (r, g, b) in zip(cluster_points, cluster_colors)] 

                # pc2_msg = pc2.create_cloud(
                #     header=msg.header,
                #     fields=fields,
                #     points=points_with_rgb
                # )        # Publish new colored point cloud
                # self._pub.publish(pc2_msg)    
        
    def pack_rgb(self, r, g, b):
        rgb_int = (int(r * 255) << 16) | (int(g * 255) << 8) | int(b * 255)
        return struct.unpack('f', struct.pack('I', rgb_int))[0]
    

    def broadcast_tf_map(self, stamp, frame, point, classify):

        time = rclpy.time.Time().from_msg(stamp)
        # Wait for the transform asynchronously
        tf_base_map_future = self.tf_buffer.wait_for_transform_async(
            target_frame='map',
            source_frame=frame,
            time=time
        )

        # Spin until transform found or `timeout_sec` seconds has passed
        rclpy.spin_until_future_complete(self, tf_base_map_future, timeout_sec=1)

        try:
            tf_base_map = self.tf_buffer.lookup_transform(
                'map',
                frame,
                time)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {frame} to {'map'}: {ex}')
            return


        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = classify  

        # Populate the transform with the transformed position and rotation
        transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point, tf_base_map)
        t.transform.translation.x = transformed_point_stamped.point.x
        t.transform.translation.y = transformed_point_stamped.point.y
        t.transform.translation.z = transformed_point_stamped.point.z 

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transform to the broadcaster
        self._tf_broadcaster.sendTransform(t)

        self.generate_map(classify, t.transform.translation.x, t.transform.translation.y, t.transform.rotation.x)


    def generate_map(self, classify, x, y, a):
        """
        Writes a new log entry only if a similar entry (based on Euclidean distance) does not already exist.
        
        :param classify: The object classification ('box' or 'object')
        :param x: X-coordinate
        :param y: Y-coordinate
        :param a: Angle
        :param threshold: The minimum Euclidean distance to consider two points as duplicates
        """

        label = 'B' if 'box' in classify else 'O'
        new_entry = (label, x, y, a)

        # Check for duplicate entry
        if label == 'B':
            threshold = 0.25
        else:
            threshold = 0.2

        if self.is_duplicate(new_entry, threshold):
            return 

        # Append new entry to the file
        log_entry = f"{label} {x:.2f} {y:.2f} {a:.0f}\n"
        with open(self.file, "a") as file:
            file.write(log_entry)

        self.get_logger().info(f"Logged: {log_entry.strip()}")


    def is_duplicate(self, new_entry, threshold):
        """
        Searches for a duplicate entry. If found, it overwrites the line in place.
        
        :param new_entry: (label, x, y, a) tuple representing the new entry
        :param threshold: The minimum distance to consider two points as duplicates
        :return: True if a match was found and updated, otherwise False
        """
        label, new_x, new_y, new_a = new_entry

        if not os.path.exists(self.file):
            return False  # No file means no duplicates exist

        with open(self.file, "r+") as file:
            position = 0  # Track file position for seeking
            for line in file:
                parts = line.strip().split(" ")
                if len(parts) < 4:
                    position += len(line)
                    continue  # Skip malformed lines

                existing_label, existing_x, existing_y, existing_a = parts[:4]
                existing_x, existing_y, existing_a = float(existing_x), float(existing_y), float(existing_a)

                if existing_label == label:
                    distance = math.sqrt((new_x - existing_x) ** 2 + (new_y - existing_y) ** 2)

                    if distance < threshold:
                        # Compute the averaged values
                        avg_x = (existing_x + new_x) / 2
                        avg_y = (existing_y + new_y) / 2
                        avg_a = round((existing_a + new_a) / 2)

                        updated_entry = f"{label} {avg_x:.2f} {avg_y:.2f} {avg_a}\n"

                        # Seek back to the position and overwrite
                        file.seek(position)
                        file.write(updated_entry)
                        self.get_logger().info(f"Overwrite: {updated_entry.strip()}")

                        return True  # Successfully overwritten

                position += len(line)  # Update file position

        return False  # No duplicate found


def main():
    if os.path.exists("map_file.txt"):
        os.remove("map_file.txt")
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
