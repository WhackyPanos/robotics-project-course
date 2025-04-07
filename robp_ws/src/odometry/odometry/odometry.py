#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose2D
from sensor_msgs.msg import Imu, MagneticField # MagneticField is msg for /imu_mag
from collections import deque 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, '/path', 100)
        self._pose_pub = self.create_publisher(Pose2D, '/odom_pose', 10)

        # Store the path here
        self._path = Path()
        self.pose_msg = Pose2D()

        # Subscribe to encoder topic and call callback function on each recieved message
        self.create_subscription(
            Encoders, '/motor/encoders', self.encoder_callback, 10)
        
        self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        
        #self.localization_transform_trigger = self.create_subscription(
        #   PointCloud2, "/icp/global_point_cloud", self.localization_transform_trigger, qos_profile = 10)
        
        
        self.init_imu_yaw = None
        self.prev_imu_yaw = None
        self.imu_yaw = None
        self.yaw_buffer = deque(maxlen=5)

        # 2D pose
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # keep encoder ticks
        self.past_ticks_left = 0
        self.past_ticks_right = 0
        self.current_ticks_left = 0
        self.current_ticks_right = 0

        # publish initial transform between map and odom
        self.tf_broadcaster_init = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.parent = 'odom'
        self.child = 'map'
        
        #self.publish_initial_transform()
        #self.count = 0

        # Retrieve transform and if it's null, publish a new one
        #self.transform_timer = self.create_timer(0.25, self.publish_transform_until_localization)

    def publish_initial_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg() #rclpy.time.Time(seconds=0).to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = self.child

        # Set translation to zero
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set rotation to zero (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0  # Identity rotation

        self.get_logger().info('Publishing initial map to odom frame (in odom node)')
        self.get_logger().info('Sending transform (odom node)')
        self.tf_broadcaster_init.sendTransform(t)


    def publish_transform_until_localization(self):
        #self.get_logger().info('Checking if localization published transform')
        transform = self.tf_buffer.lookup_transform(
            self.child,  # Target frame
            self.parent,  # Source frame
            rclpy.time.Time(seconds=0.0),  # Get the latest available transform
            timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout for lookup
        )
        if transform.transform.translation.x != 0.0 or transform.transform.translation.y != 0.0:
            pass
        else:
            self.publish_initial_transform()
        return

    def localization_transform_trigger(self, msg):
        if self.count == 0:
            self.get_logger().info('Killing map -> odom "static" transformn')
            self.transform_timer.cancel()
            self.count = 1            


    def imu_callback(self, msg: Imu):
        alpha = 0.5  # Weight for bias to past or new value of angle 

        quat = msg.orientation
        _, _, temp_imu_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        # # Sets initaial angle to 0
        # if self.init_imu_yaw is None:
        #     self.init_imu_yaw = temp_imu_yaw
        # temp_imu_yaw = self.init_imu_yaw - temp_imu_yaw

        # temp_imu_yaw = - temp_imu_yaw # Comment out if you uncomment above code

        # if self.prev_imu_yaw is not None:
        #     self._yaw = alpha * temp_imu_yaw + (1 - alpha) * self.prev_imu_yaw
        # else:
        #     self._yaw = temp_imu_yaw

        # # Store in the buffer
        # self.yaw_buffer.append(temp_imu_yaw)

        # # Compute the moving average
        # self._yaw = np.mean(self.yaw_buffer)

        # # Compute weighted moving average
        # weights = np.linspace(0.2, 1.0, len(self.yaw_buffer))  # Increasing weights
        # weights /= weights.sum()  # Normalize weights

        # self._yaw = np.dot(weights, list(self.yaw_buffer))  # Weighted sum

        # self.prev_imu_yaw = self._yaw

        if self.prev_imu_yaw is None:
            self.prev_imu_yaw = temp_imu_yaw

        self.imu_yaw = temp_imu_yaw



    def encoder_callback(self, msg: Encoders):
        """Takes encoder readings and updates the odometry.
        This function is called every time the encoders are updated (i.e., when a message is published on the '/motor/encoders' topic).
        Your task is to update the odometry based on the encoder data in 'msg'. You are allowed to add/change things outside this function.
        Keyword arguments:
        msg -- An encoders ROS message. To see more information about it 
        run 'ros2 interface show robp_interfaces/msg/Encoders' in a terminal.
        """

        if self.imu_yaw is None:
            self.get_logger().warning("Wait for IMU data!")
            return

        # The kinematic parameters for the differential configuration
        dt = 50 / 1000
        ticks_per_rev = 48 * 64
        wheel_radius = 0.0485  # TODO: Fill in
        base = 0.3125  # TODO: Fill in
        K = 2*np.pi/ticks_per_rev

        # Ticks since last message
        self.current_ticks_left = msg.encoder_left
        self.current_ticks_right = msg.encoder_right
        delta_ticks_left = self.current_ticks_left - self.past_ticks_left
        delta_ticks_right = self.current_ticks_right - self.past_ticks_right


        # TODO: Fill in
        D = wheel_radius/2 * K * (delta_ticks_right + delta_ticks_left)
        # delta_theta = wheel_radius/base * K * (delta_ticks_right - delta_ticks_left)
        self._yaw += self.prev_imu_yaw - self.imu_yaw + math.pi/(180*500)
        self.prev_imu_yaw = self.imu_yaw
        self._x = self._x + D * np.cos(self._yaw) # TODO: Fill in
        self._y = self._y + D * np.sin(self._yaw) # TODO: Fill in
        # self._yaw = self._yaw + delta_theta # TODO: Fill in
        
        #stamp = self.get_clock().now() # TODO: Fill in
        self.stamp = msg.header.stamp

        self.broadcast_transform(self.stamp, self._x, self._y, self._yaw)
        self.publish_path(self.stamp, self._x, self._y, self._yaw)

        # publishing 2D pose
        self.pose_msg.x = self._x
        self.pose_msg.y = self._y
        self.pose_msg.theta = self._yaw

        self._pose_pub.publish(self.pose_msg)
        
        # store as past ticks
        self.past_ticks_left = self.current_ticks_left
        self.past_ticks_right = self.current_ticks_right


    def broadcast_transform(self, stamp, x, y, yaw):
        """Takes a 2D pose and broadcasts it as a ROS transform.
        Broadcasts a 3D transform with z, roll, and pitch all zero. 
        The transform is stamped with the current time and is between the frames 'odom' -> 'base_link'.
        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)

    def publish_path(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the path and publishes the whole path.
        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)
        
        self._path_pub.publish(self._path)


def main():
    rclpy.init()
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()

