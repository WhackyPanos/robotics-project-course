#!/usr/bin/env python
import os
import math
import string
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_geometry_msgs


from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import ctypes
import struct


class Detection(Node):

    def __init__(self):
        super().__init__('detection')
        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)

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

        # Convert ROS -> NumPy
        print("detection callback called")

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

        box_detected = False
        # Filter points based on distance and height
        box_mask = (points[:, 2] < 1.0) & (points[:, 1] < 0.03 ) & (points[:,1] > 0)
        box_points = points[box_mask]
        box_colors = colors[box_mask]


        # # define fields and points to creare a Pointcloud2 message later on
        # fields = [
        #     pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),  # X-coordinate (float32)
        #     pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),  # Y-coordinate (float32)
        #     pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),  # Z-coordinate (float32)
        #     pc2.PointField(name='r', offset=12, datatype=pc2.PointField.UINT8, count=1),   # Red channel (uint8)
        #     pc2.PointField(name='g', offset=13, datatype=pc2.PointField.UINT8, count=1),   # Green channel (uint8)
        #     pc2.PointField(name='b', offset=14, datatype=pc2.PointField.UINT8, count=1)    # Blue channel (uint8)
        # ]

        # # Assuming filtered_coords is the filtered point coordinates and filtered_rgb the RGB values
        # point_dtype = np.dtype([
        #                         ('x', np.float32),  # X-coordinate (float32)
        #                         ('y', np.float32),  # Y-coordinate (float32)
        #                         ('z', np.float32),  # Z-coordinate (float32)
        #                         ('r', np.uint8),    # Red channel (uint8)
        #                         ('g', np.uint8),    # Green channel (uint8)
        #                         ('b', np.uint8)     # Blue channel (uint8)
        #                     ])

        # n = len(filtered_colors[:, 0])  # Number of points
        # ds_points = np.zeros(n, dtype=point_dtype)  # One entry per point

        # # Populate the filtered_points array with coordinates and colors
        # for i in range(n):
        #     ds_points[i]['x'] = filtered_points[i, 0]  # X-coordinate
        #     ds_points[i]['y'] = filtered_points[i, 1]  # Y-coordinate
        #     ds_points[i]['z'] = filtered_points[i, 2]  # Z-coordinate
        #     ds_points[i]['r'] = int(filtered_colors[i, 0] * 255)  # Red channel (scaled to 0-255)
        #     ds_points[i]['g'] = int(filtered_colors[i, 1] * 255)  # Green channel (scaled to 0-255)
        #     ds_points[i]['b'] = int(filtered_colors[i, 2] * 255)  # Blue channel (scaled to 0-255)

            # print(f"Detected point {i} at coordinates {filtered_coords[i]}, with color {filtered_rgb[i]}")

        # # change header of message
        # filtered_msg_header = msg.header
        stamp = msg.header.stamp
        frame = msg.header.frame_id
        # filtered_msg_header.frame_id = "camera_link"

        # # create the PointCloud2 message and publish it
        # filtered_msg = pc2.create_cloud(filtered_msg_header, fields, ds_points)
        # self._pub.publish(filtered_msg)

        # Detect objects based on color
        red_mask = (box_colors[:, 0] > 0.8) & (box_colors[:, 1] < 0.4) 
        green_mask = (box_colors[:, 0] < 0.1) & (box_colors[:, 1] > 0.4)
        gray_mask = (box_colors[:,0] > 0.3) & (box_colors[:,0] < 0.7) & \
                    (box_colors[:,1] > 0.3) & (box_colors[:,1] < 0.7) & \
                    (box_colors[:,2] > 0.3) & (box_colors[:,2] < 0.7)
        
        # self.get_logger().info(f"Number of gray points: {box_colors[gray_mask].shape}")
        # if np.any(box_colors):
        #     self.get_logger().info(f"Min: {np.min(box_colors[:,0]), np.min(box_colors[:,1]), np.min(box_colors[:,2])}")
        #     self.get_logger().info(f"Max: {np.max(box_colors[:,0]), np.max(box_colors[:,1]), np.max(box_colors[:,2])}")
        #     self.get_logger().info(f"Mean: {np.mean(box_colors[:,0]), np.mean(box_colors[:,1]), np.mean(box_colors[:,2])}")
        if np.size(box_points[:,0]) > 250:
            self.get_logger().info("Grey box detected!")
            box_detected = True
            box_centre = np.mean(box_points, axis=0)           
            box_point_stamped = PointStamped()
            box_point_stamped.header.stamp = stamp
            box_point_stamped.header.frame_id = frame
            box_point_stamped.point.x, box_point_stamped.point.y, box_point_stamped.point.z = box_centre

            self.broadcast_tf_map(stamp, frame, box_point_stamped, 'box')


        object_mask = (points[:, 2] < 1.0) & (points[:, 1] < 0.08 ) & (points[:,1] > 0.03)
        object_points = points[object_mask]
        object_colors = colors[object_mask]
        
        # self.get_logger().info(f"Number of object points: {object_colors.shape}")
        # if np.any(object_colors):
        #     self.get_logger().info(f"Min: {np.min(object_colors[:,0]), np.min(object_colors[:,1]), np.min(object_colors[:,2])}")
        #     self.get_logger().info(f"Max: {np.max(object_colors[:,0]), np.max(object_colors[:,1]), np.max(object_colors[:,2])}")
        #     self.get_logger().info(f"Mean: {np.mean(object_colors[:,0]), np.mean(object_colors[:,1]), np.mean(object_colors[:,2])}")
        if(np.size(object_points[:,0])) > 250 and not box_detected:
            self.get_logger().info("Object detected!")

            object_centre = np.mean(object_points, axis=0)           
            object_point_stamped = PointStamped()
            object_point_stamped.header.stamp = stamp
            object_point_stamped.header.frame_id = frame
            object_point_stamped.point.x, object_point_stamped.point.y, object_point_stamped.point.z = object_centre

            self.broadcast_tf_map(stamp, frame, object_point_stamped, 'object')
    

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
