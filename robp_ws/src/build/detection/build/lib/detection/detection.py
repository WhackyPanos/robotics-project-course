#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import ctypes
import struct


class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)

    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """

        # Convert ROS -> NumPy

        # uses the read_points_numpy function to convert the binary point cloud data into a NumPy array. 
        # Each row represents a point, and points contains the XYZ coordinates.
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        # Extract the XYZ coordinates and RGB values
        coords = points[:, :3]  # x, y, z

        # Each point has a packed 32-bit color value. The code extracts the RGB components by:
        #   Packing the float color (struct.pack).
        #   Converting to an integer (struct.unpack).
        #   Shifting and masking to extract the red, green, and blue components.
        for idx, x in enumerate(gen):
            c = x[3] # eg: x = [-1.9799935e+00 -1.7417923e+00  5.5220003e+00  6.3787709e-39]
            s = struct.pack('>f', c) # eg: s = b'\x00\xbc\xa6\xa8'
            i = struct.unpack('>l', s)[0] # eg: i = 12363432
            pack = ctypes.c_uint32(i).value 
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)
            #eg: pack = 10131083, colors = [[158 130 100] [135 102  73]  [126  87  67]]... [  0   0   0] [  0   0   0] [  0   0   0]], #print(f'pack = {pack}, colors = {colors}')
                            
        colors = colors.astype(np.float32) / 255

        # Define thresholds
        dthresh = 0.9
        desired_color = 0.8
        rem_color = 0.5

        # Filter points farther than 0.9m/below ground and points that don't have strong red or green components
        # Distance filter: Keep points closer than 0.9m
        close_mask = coords[:, 2] < dthresh

        # Color filter: Keep red or green points based on a threshold
        red_mask = (colors[:, 0] > desired_color) & (colors[:,1] < rem_color) & (colors[:,2] < rem_color)
        green_mask = (colors[:, 1] > desired_color) & (colors[:,0] < rem_color+0.2) & (colors[:,2] < rem_color+0.2)
        color_mask = red_mask | green_mask  

        # Combine masks
        filter_mask = close_mask & color_mask

        # Apply mask
        filtered_coords = coords[filter_mask]
        filtered_rgb = colors[filter_mask]

        #print(f'length filtered rgb = {len(filtered_rgb[:,0])} and length filtered coords= {len(filtered_coords[:,0])}')

        # define fields and points to creare a Pointcloud2 message later on
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),  # X-coordinate (float32)
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),  # Y-coordinate (float32)
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),  # Z-coordinate (float32)
            pc2.PointField(name='r', offset=12, datatype=pc2.PointField.UINT8, count=1),   # Red channel (uint8)
            pc2.PointField(name='g', offset=13, datatype=pc2.PointField.UINT8, count=1),   # Green channel (uint8)
            pc2.PointField(name='b', offset=14, datatype=pc2.PointField.UINT8, count=1)    # Blue channel (uint8)
        ]

        # Assuming filtered_coords is the filtered point coordinates and filtered_rgb the RGB values
        point_dtype = np.dtype([
                                ('x', np.float32),  # X-coordinate (float32)
                                ('y', np.float32),  # Y-coordinate (float32)
                                ('z', np.float32),  # Z-coordinate (float32)
                                ('r', np.uint8),    # Red channel (uint8)
                                ('g', np.uint8),    # Green channel (uint8)
                                ('b', np.uint8)     # Blue channel (uint8)
                            ])
        n = len(filtered_rgb[:, 0])  # Number of points


        filtered_points = np.zeros(n, dtype=point_dtype)  # One entry per point

        # Populate the filtered_points array with coordinates and colors
        for i in range(n):
            filtered_points[i]['x'] = filtered_coords[i, 0]  # X-coordinate
            filtered_points[i]['y'] = filtered_coords[i, 1]  # Y-coordinate
            filtered_points[i]['z'] = filtered_coords[i, 2]  # Z-coordinate
            filtered_points[i]['r'] = int(filtered_rgb[i, 0] * 255)  # Red channel (scaled to 0-255)
            filtered_points[i]['g'] = int(filtered_rgb[i, 1] * 255)  # Green channel (scaled to 0-255)
            filtered_points[i]['b'] = int(filtered_rgb[i, 2] * 255)  # Blue channel (scaled to 0-255)

            print(f"Detected point {i} at coordinates {filtered_coords[i]}, with color {filtered_rgb[i]}")

        # change header of message
        filtered_msg_header = msg.header
        filtered_msg_header.frame_id = "camera_link"

        # create the PointCloud2 message and publish it
        filtered_msg = pc2.create_cloud(filtered_msg_header, fields, filtered_points)
        self._pub.publish(filtered_msg)
        

def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
