#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_matrix, unit_vector
from tf2_ros import TransformException
import numpy as np
from math import acos, pi


class Localization(Node):
    def __init__(self):
        super().__init__('localization_node') 

        self.pc_transform_subscriber = self.create_subscription(
            msg_type = TransformStamped,
            topic = "/icp/transform",
            callback =  self.localization_transform,
            qos_profile=10)
        # self.pc_subscriber = self.create_subscription(
        #     msg_type = PointCloud2,
        #     topic = "/icp/transformed_point_cloud",
        #     callback =  None,
        #     qos_profile=10)
        
        self.icp_master_publisher = self.create_publisher(
            msg_type = String,
            topic = "/icp/type",
            qos_profile = 10)
        self.transform_publisher = self.create_publisher(
            msg_type = TransformStamped,
            topic = 'tf2',
            qos_profile = 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.to_frame_rel = 'odom'
        self.from_frame_rel = 'map'
        self.old_stamp = None
        self.old_q = None
        self.old_transform_x = 0
        self.old_transform_y = 0
        self.old_transform_z = 0
        self.icp_msg = String()

        # publish identity transformation while ICP does not work
        # self.publish_initial_transform()
        # self.count = 0
        self.transform_timer = self.create_timer(0.1, self.publish_transform_until_localization)

        
    def icp_master(self):
        """ Method that publishes a string message so ICP can be performed. It is supposed to be controlled by BT"""
        icp_stype = 'Normal' #change later
        self.icp_msg.data = icp_stype
        self.icp_master_publisher.publish(self.icp_msg)
            

    def localization_transform(self, msg):
        """ map -> odom transform update: retrieve "old" transform and update with with the PCL transform"""
        #self.get_logger().warn("Transform from ICP received!")
        if self.old_stamp is None:
            self.old_stamp = msg.header.stamp 
            return
        
        # retrieve old transform (with correct timestamp) see documentation https://docs.ros.org/en/jade/api/tf/html/python/tf_python.html
        try:
            t_old = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'odom', # Source frame
                self.old_stamp, )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}. Using most recent transforms')
            t_old = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'odom', # Source frame
            rclpy.time.Time(seconds=0))
        
        # create and broadcast new transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp 
        t.header.frame_id = 'odom'
        t.child_frame_id = 'map'

        # Correct translation composition: apply old rotation to the new translation
        delta_t = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        R_old = quaternion_matrix([t_old.transform.rotation.x, t_old.transform.rotation.y, t_old.transform.rotation.z, t_old.transform.rotation.w])[:3, :3]
        t_old_translation = np.array([t_old.transform.translation.x, t_old.transform.translation.y, t_old.transform.translation.z])
        new_translation = t_old_translation + R_old.dot(delta_t)
        t.transform.translation.x  = new_translation[0]
        t.transform.translation.y  = new_translation[1]
        t.transform.translation.z  = new_translation[2]

        q = quaternion_multiply([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w],
                                [t_old.transform.rotation.x, t_old.transform.rotation.y, t_old.transform.rotation.z, t_old.transform.rotation.w])
        #q = unit_vector(q)
        if self.old_q is None:
            self.old_q = q

        # check if incoming quaternion introduced ~180 of rotation
        q = list(q)
        for i, q_component in enumerate(q):
            if q_component*self.old_q[i] < 0:
                q[i] = -q_component

        self.get_logger().info(f"icp quaternion = {[msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]}")
        self.get_logger().info(f"previous transform quaternion = {[t_old.transform.rotation.x, t_old.transform.rotation.y, t_old.transform.rotation.z, t_old.transform.rotation.w]}")
        self.get_logger().info(f"resulting quaternion = {q}")

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]


        # Send the transformation
        self.get_logger().info(f'Publishing transform between map and odom (localization node)')
        self.tf_broadcaster.sendTransform(t)
        self.old_q = q
        self.old_stamp = t.header.stamp

        # TODO: get odom pose and transform to map pose

    def publish_initial_transform(self):
        #self.get_logger().info('Publishing identity transform: ICP was not performed yet')
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg() #rclpy.time.Time(seconds=0).to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "map"

        # Set translation to zero
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set rotation to zero (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0  # Identity rotation

        self.tf_broadcaster.sendTransform(t)


    def publish_transform_until_localization(self):
        #self.get_logger().info('Checking if localization published transform')
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",  # Target frame
                "odom",  # Source frame
                rclpy.time.Time(seconds=0.0),  # Get the latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout for lookup
            )
            if transform.transform.translation.x != 0.0 or transform.transform.translation.y != 0.0 or transform.transform.rotation.z != 0.0:
                self.get_logger().info('Killing map -> odom "static" transform')
                self.transform_timer.cancel()
            else:
                self.publish_initial_transform()
            return
        except:
            self.publish_initial_transform()

def main():
    """ Function to be run if localization/icp is to be run all the time"""
    rclpy.init()
    node = Localization()
    icp_time = 3 #every 1 s
    node.create_timer(icp_time, node.icp_master)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()

