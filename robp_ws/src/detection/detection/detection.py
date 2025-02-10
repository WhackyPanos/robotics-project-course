#!/usr/bin/env python
import math
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

        # # Define thresholds
        # dthresh = 0.9
        # desired_color = 0.7
        # rem_color = 0.5

        # # Filter points farther than 0.9m/below ground and points that don't have strong red or green components
        # # Distance filter: Keep points closer than 0.9m
        # close_mask = coords[:, 2] < dthresh

        # # Color filter: Keep red or green points based on a threshold
        # red_mask = (colors[:, 0] > desired_color) & (colors[:,1] < rem_color) & (colors[:,2] < rem_color)
        # green_mask = (colors[:, 1] > desired_color) & (colors[:,0] < rem_color) & (colors[:,2] < rem_color)
        # color_mask = red_mask | green_mask  

        # # Combine masks
        # filter_mask = close_mask & color_mask

        # # Apply mask
        # filtered_coords = coords[filter_mask]
        # filtered_rgb = colors[filter_mask]

        # #print(f'length filtered rgb = {len(filtered_rgb[:,0])} and length filtered coords= {len(filtered_coords[:,0])}')

        # Filter points based on distance and height
        points_mask = (points[:, 2] < 0.9) & (points[:, 1] < 0.095)
        filtered_points = points[points_mask]
        filtered_colors = colors[points_mask]


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

        n = len(filtered_colors[:, 0])  # Number of points
        ds_points = np.zeros(n, dtype=point_dtype)  # One entry per point

        # Populate the filtered_points array with coordinates and colors
        for i in range(n):
            ds_points[i]['x'] = filtered_points[i, 0]  # X-coordinate
            ds_points[i]['y'] = filtered_points[i, 1]  # Y-coordinate
            ds_points[i]['z'] = filtered_points[i, 2]  # Z-coordinate
            ds_points[i]['r'] = int(filtered_colors[i, 0] * 255)  # Red channel (scaled to 0-255)
            ds_points[i]['g'] = int(filtered_colors[i, 1] * 255)  # Green channel (scaled to 0-255)
            ds_points[i]['b'] = int(filtered_colors[i, 2] * 255)  # Blue channel (scaled to 0-255)

            # print(f"Detected point {i} at coordinates {filtered_coords[i]}, with color {filtered_rgb[i]}")

        # change header of message
        filtered_msg_header = msg.header
        stamp = msg.header.stamp
        frame = msg.header.frame_id
        filtered_msg_header.frame_id = "camera_link"

        # create the PointCloud2 message and publish it
        filtered_msg = pc2.create_cloud(filtered_msg_header, fields, ds_points)
        self._pub.publish(filtered_msg)

        # Detect objects based on color
        red_mask = (filtered_colors[:, 0] > 0.8) & (filtered_colors[:, 1] < 0.4) 
        green_mask = (filtered_colors[:, 0] < 0.1) & (filtered_colors[:, 1] > 0.4)
        gray_mask = (filtered_colors[:,0] > 0.45) & (filtered_colors[:,0] < 0.55) & \
                    (filtered_colors[:,1] > 0.45) & (filtered_colors[:,1] < 0.55) & \
                    (filtered_colors[:,2] > 0.45) & (filtered_colors[:,2] < 0.55)
        
        if len(filtered_colors[gray_mask]) > 500:
            self.get_logger().info("Grey box detected!")
            print("Number of points", filtered_colors[gray_mask])
            


        # # Check if red sphere is detected
        # if np.any(red_mask):
        #     self.get_logger().info("Red sphere detected!")
            
        #     red_values = filtered_colors[red_mask, 0]

        #     # Find the maximum red value
        #     max_red_value = np.max(red_values)

        #     # Find all indices where the red value equals the maximum
        #     max_red_indices = np.where(red_values == max_red_value)[0]

        #     # If there are multiple points, select the middle one
        #     if len(max_red_indices) > 1:
        #         red_pixel_index = max_red_indices[len(max_red_indices) // 2]
        #     else:
        #         red_pixel_index = max_red_indices[0]
            
        #     # Get the 3D coordinates of this point
        #     red_points = filtered_points[red_mask]
        #     red_center = red_points[red_pixel_index]

        #     red_point_stamped = PointStamped()
        #     red_point_stamped.header.stamp = stamp
        #     red_point_stamped.header.frame_id = frame
        #     red_point_stamped.point.x, red_point_stamped.point.y, red_point_stamped.point.z = red_center

        #     self.broadcast_tf_map(stamp, frame, red_point_stamped, 'red')


        # # Check if green cube is detected
        # if np.any(green_mask):
        #     self.get_logger().info("Green cube detected!")

        #     # Extract green channel of the filtered points
        #     green_values = filtered_colors[green_mask, 1]

        #     # Find the maximum green value
        #     max_green_value = np.max(green_values)

        #     # Find all indices where the green value equals the maximum
        #     max_green_indices = np.where(green_values == max_green_value)[0]

        #     # If there are multiple points, select the middle one
        #     if len(max_green_indices) > 1:
        #         green_pixel_index = max_green_indices[len(max_green_indices) // 2]
        #     else:
        #         green_pixel_index = max_green_indices[0] 
            
        #     # Get the 3D coordinates of this point
        #     green_points = filtered_points[green_mask]
        #     green_center = green_points[green_pixel_index]

        #     green_point_stamped = PointStamped()
        #     green_point_stamped.header.stamp = stamp
        #     green_point_stamped.header.frame_id = frame
        #     green_point_stamped.point.x, green_point_stamped.point.y, green_point_stamped.point.z = green_center

        #     self.broadcast_tf_map(stamp, frame, green_point_stamped, 'green')

        


    def broadcast_tf_map(self, stamp, frame, point, color):

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
        t.header.frame_id = 'map'  # parent frame is the map frame
        t.child_frame_id = f'object/{color}'  # child frame is the ArUco marker's frame

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
