#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from geometry_msgs.msg import Pose2D, PointStamped, PoseStamped
from std_msgs.msg import Bool, String
import numpy as np
from math import sqrt, cos, sin, pi, sqrt, atan2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from geometry_msgs.msg import PoseStamped, Pose, PointStamped, PoseArray, Point, Pose2D, Twist



class UpdateObjectList(py_trees.behaviour.Behaviour, Node):
    def __init__(self, obj_list, box_list ,name ='UpdateObjectList'):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        self.obj_list = obj_list
        self.box_list = box_list
        self.robot_pos = np.array([0,0])

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs["node"]
        # TODO: transform to map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.to_frame_rel = 'map'
        self.from_frame_rel = 'odom'

        self.robot_pos_sub = self.node.create_subscription(Pose2D, '/odom_pose', self.get_robot_pos_callback, 10)
        self.need_next_object_sub = self.node.create_subscription(String, '/next_goal/object/need', self.need_next_object_callback, 10)
        self.update_object_list_sub = self.node.create_subscription(Bool, '/next_goal/object/update', self.update_object_list_callback, 10)

        self.next_goal_pub = self.node.create_publisher(PoseStamped,'/motion/goal', 10 )
        self.need_next_object = 'Object' # at the beginning, we want to pick objects

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        self.node.get_logger().info(f"Objects list is now: {self.obj_list}")
        if self.need_next_object is not None:
            if self.need_next_object == 'Object':
                points_list = self.obj_list
            else:
                points_list = self.box_list
            # compute closest object
            distances = np.array([sqrt((self.robot_pos[0]-obj[1])**2 + (self.robot_pos[1]-obj[2])**2) for obj in points_list])
            closest_obj = points_list[np.argmin(distances)]

            # publish goal point (object to pick)
            msg = PoseStamped()
            msg.pose.position.x = closest_obj[1]
            msg.pose.position.y = closest_obj[2]
            self.node.get_logger().info(f"Closest stuff: {msg.pose.position.x , msg.pose.position.y}")
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            self.next_goal_pub.publish(msg)

            self.need_next_object = None
        return py_trees.common.Status.SUCCESS


    def get_robot_pos_callback(self, msg):
        """ Get robot position in odom frame and convert to map frame. If transform fails, use odom frame since
        we just need a rough estimate of the closest object """
    
        try:
            t = self.tf_buffer.lookup_transform(
                self.to_frame_rel,
                self.from_frame_rel,
                rclpy.time.Time(seconds=0))
            stamp = t.header.stamp
            
            point_odom = PointStamped()
            point_odom.header.frame_id = 'odom'
            point_odom.header.stamp = stamp
            point_odom.point.x = msg.x
            point_odom.point.y = msg.y
            point_odom.point.z = 0.0

            point_map = self.tf_buffer.transform(point_odom, self.to_frame_rel)
            self.robot_pos[0] = point_map.point.x
            self.robot_pos[1] = point_map.point.y

        except TransformException as ex:
            self.robot_pos[0] = msg.x
            self.robot_pos[1] = msg.y


        return

    def need_next_object_callback(self, msg):
        self.need_next_object = msg.data
        self.node.get_logger().info(f"Next thing to go in motion: {msg.data}")

    def update_object_list_callback(self, msg):
        # x = msg.point.x
        # y = msg.point.y
        # self.obj_list = [obj for obj in self.obj_list if obj[1] != x and obj[2] != y]

        # remove the closest object
        if msg.data:
            distances = np.array([sqrt((self.robot_pos[0]-obj[1])**2 + (self.robot_pos[1]-obj[2])**2) for obj in self.obj_list])
            closest_obj_idx = np.argmin(distances)
            self.obj_list.pop(closest_obj_idx)


class ArmTaskSucceeded(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name = 'ArmTaskSucceeded'):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        self.arm_task = 'pick' # can be either 'pick' or 'place'
    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.picklift_sub = self.node.create_subscription(Bool, '/picklift/succeded', self.picklift_callback, 10)
        self.next_goal = self.node.create_publisher(String, '/next_goal/object/need', 10)
        self.update_obj_list_pub = self.node.create_publisher(Bool,'/next_goal/object/update', 10)
        self.msg = String()

    def update(self):
        # Note: hope the behavior tick is slower than the subscriber callback, might need changes
        self.node.get_logger().info(f"Next thing to go to is {self.msg.data} and we are in the {self.arm_task} task")
        self.next_goal.publish(self.msg)
        return py_trees.common.Status.FAILURE

    def picklift_callback(self, msg):
        if not msg.data: # pick_lift failed, remove object from list and try same movement (pick object or place on box)
            self.msg.data = 'Object' if self.arm_task == 'pick' else 'Box'
            pass
        else: # pick lift succeeded, remove object from list and try placing now
            self.msg.data = 'Box' if self.arm_task == 'pick' else 'Object'
            self.arm_task = 'place' if self.msg.data == 'Box' else 'pick'
            pass
        self.update_obj_list_pub.publish(Bool(data=True)) #remove closest object from objects list
        return
    
class Adjust(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name = 'Adjust'):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        self.X_arm_cam, self.Y_arm_cam = [],  []
        self.X_obj, self.Y_obj = None, None
        self.yaw_msg_sent, self.distance_msg_sent = False, False
        self.linear_velocity = 0.05
        self.angular_velocity = 0.05
        self.min = None

        self.vel_cmd = Twist()
        self.vel_cmd.linear.y = 0.0
        self.vel_cmd.linear.z = 0.0
        self.vel_cmd.angular.x = 0.0
        self.vel_cmd.angular.y = 0.0

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.next_goal_pub = self.node.create_subscription(PoseArray, '/arm_camera/points',  self.get_next_goal_arm_cam_callback, 10 )

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            

    def update(self):
        # Note: hope the behavior tick is slower than the subscriber callback, might need changes
        yaw_adjusted = self.adjust_yaw()
        if yaw_adjusted:
            return py_trees.common.Status.FAILURE #return failure to repeat tuck, detect and IK
        else:
            return py_trees.common.Status.RUNNING


    def get_next_goal_arm_cam_callback(self, msg):
        if len(self.X_arm_cam) == 0:
            for pose in msg.poses:
                self.X_arm_cam.append(pose.position.x)
                self.Y_arm_cam.append(pose.position.y)
            self.min = 10000
            idx =  0
            for i in range(len(self.X_arm_cam)):
                if sqrt(self.X_arm_cam[i]**2 + self.X_arm_cam[i]**2) < self.min:
                    self.min = sqrt(self.X_arm_cam[i]**2 + self.X_arm_cam[i]**2)
                    idx = i
            self.X_obj = self.X_arm_cam[idx]
            self.Y_obj = self.Y_arm_cam[idx]
            self.node.get_logger().info(f"Closest object is at {min} [m]: ")
    
    def adust_distance(self):
        pass

    def adjust_yaw(self):
        if not self.yaw_msg_sent:
            self.init_time = self.get_clock().now().nanoseconds / 1e9
            desired_theta = atan2(-self.X_obj, -self.Y_obj)
            self.vel_cmd.angular.z = self.angular_velocity
            delta_t = desired_theta/self.vel_cmd.angular.z
            self.cmd_vel_publisher.publish(self.vel_cmd)
            self.yaw_msg_sent = True
            self.node.get_logger().info(f"Sending adjust yaw message: closest object is {self.min} [m] away, angle = {desired_theta*180/pi} degrees")
            return False
        else:
            self.curr_time = self.get_clock().now().nanoseconds / 1e9
            if self.curr_time - self.init < delta_t:
                self.node.get_logger().info(f"Adjusting yaw")
                return False
            else:
                self.vel_cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(self.vel_cmd)
                self.node.get_logger().info(f"Yaw adjusted")
                return True