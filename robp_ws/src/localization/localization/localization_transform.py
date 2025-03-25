#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_matrix
from tf2_ros import TransformException
import numpy as np


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
        self.to_frame_rel = 'map'
        self.from_frame_rel = 'odom'
        self.old_stamp = None

        self.icp_msg = String()
        
    def icp_master(self):
        """ Method that publishes a string message so ICP can be performed. It is supposed to be controlled by BT"""
        icp_stype = 'Normal' #change later
        self.icp_msg.data = icp_stype
        self.icp_master_publisher.publish(self.icp_msg)
            

    def localization_transform(self, msg):
        """ map -> odom transform update: retrieve "old" transform and update with with the PCL transform"""
        self.get_logger().warn("Transform from ICP received!")
        if self.old_stamp is None:
            self.old_stamp = msg.header.stamp 
            return
        
        # retrieve old transform (with correct timestamp) see documentation https://docs.ros.org/en/jade/api/tf/html/python/tf_python.html
        try:
            t_old = self.tf_buffer.lookup_transform(
                self.to_frame_rel,  # Target frame
                self.from_frame_rel, # Source frame
                self.old_stamp, )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}. Using most recent transforms')
            t_old = self.tf_buffer.lookup_transform(
            self.to_frame_rel,  # Target frame
            self.from_frame_rel, # Source frame
            rclpy.time.Time(seconds=0))
        
        # create and broadcast new transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp 
        t.header.frame_id = self.from_frame_rel 
        t.child_frame_id = self.to_frame_rel

        # Correct translation composition: apply old rotation to the new translation
        delta_t = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        R_old = quaternion_matrix([t_old.transform.rotation.w, t_old.transform.rotation.x, t_old.transform.rotation.y, t_old.transform.rotation.z])[:3, :3]
        t_old_translation = np.array([t_old.transform.translation.x, t_old.transform.translation.y, t_old.transform.translation.z])
        new_translation = t_old_translation + R_old.dot(delta_t)
        t.transform.translation.x  = new_translation[0]
        t.transform.translation.y  = new_translation[1]
        t.transform.translation.z  = new_translation[2]

        q = quaternion_multiply([t_old.transform.rotation.w, t_old.transform.rotation.x, t_old.transform.rotation.y, t_old.transform.rotation.z],
                                [msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z])
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        # Send the transformation
        self.get_logger().info(f'Publishing transform between map and odom (localization node)')
        self.tf_broadcaster.sendTransform(t)
        self.old_stamp = t.header.stamp

        # TODO: get odom pose and transform to map pose


def main():
    """ Function to be run if localization/icp is to be run all the time"""
    rclpy.init()
    node = Localization()
    icp_time = 0.2 #every 1 s
    node.create_timer(icp_time, node.icp_master)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
