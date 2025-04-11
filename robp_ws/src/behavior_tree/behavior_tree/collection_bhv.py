#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from geometry_msgs.msg import Pose2D, PointStamped, PoseStamped
from std_msgs.msg import Bool, String, Int16
import numpy as np
from math import sqrt, cos, sin, pi, sqrt, atan2, dist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from geometry_msgs.msg import PoseStamped, Pose, PointStamped, PoseArray, Point, Pose2D, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy



class UpdateObjectList(py_trees.behaviour.Behaviour, Node):
    def __init__(self, obj_list, box_list ,name ='UpdateObjectList'):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        self.obj_list = obj_list
        self.box_list = box_list
        self.robot_pos = np.array([0,0])
        self.arm_task = 'pick' # can be either 'pick' or 'place'

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs["node"]
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.to_frame_rel = 'map'
        self.from_frame_rel = 'odom'

        self.robot_pos_sub = self.node.create_subscription(Pose2D, '/odom_pose', self.get_robot_pos_callback, 10)
        self.node.create_subscription(Bool, '/picklift/succeded', self.pick_place_succeeded_callback, 10)

        self.next_goal_pub = self.node.create_publisher(PointStamped,'/goal_point',
                                                        rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.next_goal_type_pub = self.node.create_publisher(String,'/goal_type',
                                                        rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.next_goal_type = 'Object' # at the beginning, we want to pick objects
        self.timer_running = False
        self.timer_finished = False

    def initialise(self):
        self.timer_finished = False
        self.timer_running = False

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        #self.node.get_logger().info(f"Objects list is now: {self.obj_list}")
        if not self.timer_running:
            if self.next_goal_type == 'Object':
                points_list = self.obj_list
            else:
                points_list = self.box_list

            # compute closest object and publish goal
            distances = np.array([sqrt((self.robot_pos[0]-obj[1])**2 + (self.robot_pos[1]-obj[2])**2) for obj in points_list])
            closest_obj = points_list[np.argmin(distances)]
            msg = PointStamped() 
            msg.point.x = closest_obj[1]
            msg.point.y = closest_obj[2]
            self.node.get_logger().info(f"Closest stuff: {msg.point.x , msg.point.y}")
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            self.next_goal_pub.publish(msg)

            # remove object from list (if we're in the picking task). Give time to return success
            if self.next_goal_type == 'Object':
                self.obj_list.pop(np.argmin(distances))
            self.timer_running = True
            self.timer = self.node.create_timer(2.0, self.wait_to_return_success)

            return py_trees.common.Status.RUNNING
        
        else:
            if not self.timer_finished:
                return py_trees.common.Status.RUNNING
            else: 
                return py_trees.common.Status.SUCCESS

    def wait_to_return_success(self):
        self.timer_finished = True

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

    def pick_place_succeeded_callback(self, msg):
        """ If pick/place task was succeeded, this callback will be called we should switch tasks. """
        if msg.data: # we only switch tasks if they succeeded
            self.next_goal_type = 'Box' if self.next_goal_type == 'Object' else 'Object'
            self.next_goal_type_pub.publish(String(data = self.next_goal_type))


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
        self.pop_obj_map_pub = self.node.create_publisher(Bool, '/object_rm', 
                                                          rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.next_goal_msg = String()

        self.count_grasping_failures_pub = self.node.create_publisher(
            Int16,'/picklift/count_grasping_failures', 10)
        self.n_fails_msg = Int16()
        self.count_grasping_failures = 0
        self.n_fails_msg.data = self.count_grasping_failures
        self.count_grasping_failures_pub.publish(self.n_fails_msg)

        self.timer_finished = False
        self.timer_started = False

        self.timer = self.node.create_timer(2.0, self.wait_for_next_goal_msg)

        # -------------------------- Picking Parameter(s) ------------------
        self.max_grasping_failures = 1
        # ------------------------------------------------------------------

    def initialise(self):
        self.timer_finished = False
        self.timer_started = False
    
    def update(self):
        # Note: hope the behavior tick is slower than the subscriber callback, might need changes
        if self.timer_finished:
            self.node.get_logger().info(f"Next thing to go to is {self.next_goal_msg.data} and we are in the {self.arm_task} task. Number of failures in this objects = {self.count_grasping_failures}")
            self.next_goal.publish(self.next_goal_msg)
            return py_trees.common.Status.FAILURE
        else:
            if not self.timer_started:
                self.timer = self.node.create_timer(2.0, self.wait_for_next_goal_msg)
                self.timer_started = True
            self.node.get_logger().info(f"Waiting to know next goal")
            return py_trees.common.Status.RUNNING
        
    def wait_for_next_goal_msg(self):
        self.timer_finished = True

    def picklift_callback(self, msg):
        """ 
        If pick_lift failed, remove object from list (publish True) and try same movement (pick object or place 
        on box) as long as we've already tried n times. Otherwise, don't remove object from the list so we can try again.
        If succeeds, remove object from list and switch to placing now"""
        self.get_logger().info(f"failures count =  {self.count_grasping_failures}")
        if not msg.data: # pick_lift failed
            self.next_goal_msg.data = 'Object' if self.arm_task == 'pick' else 'Box'
            if self.count_grasping_failures < self.max_grasping_failures:
                self.count_grasping_failures += 1
                self.n_fails_msg.data = self.count_grasping_failures
                self.count_grasping_failures_pub.publish(self.n_fails_msg)
                self.update_obj_list_pub.publish(Bool(data=False)) 
                self.get_logger().info(f"Let's try grasping this object again!")
            else:
                self.count_grasping_failures = 0
                self.n_fails_msg.data = self.count_grasping_failures
                self.count_grasping_failures_pub.publish(self.n_fails_msg)
                self.update_obj_list_pub.publish(Bool(data=True))
                self.pop_obj_map_pub.publish(Bool(data=False))
                self.get_logger().info(f"Giving up on this object!")
        else: # pick lift succeeded
            self.count_grasping_failures = 0
            self.n_fails_msg.data = self.count_grasping_failures
            self.count_grasping_failures_pub.publish(self.n_fails_msg)
            if self.arm_task == 'pick':
                self.update_obj_list_pub.publish(Bool(data=True))
                self.next_goal_msg.data = 'Box'
                self.arm_task = 'place'
            else:
                self.next_goal_msg.data = 'Object'
                self.arm_task = 'pick'
            self.pop_obj_map_pub.publish(Bool(data=True))

        #self.get_logger().info(f"Next stuff is {self.next_goal_msg.data} and task = {self.arm_task}")
        return
    
class Adjust(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name = 'Adjust'):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        self.X_arm_cam, self.Y_arm_cam = [],  []
        self.X_obj, self.Y_obj = None, None
        self.min = None

        self.position_checked = False
        self.yaw_msg_sent, self.distance_msg_sent = False, False
        self.distance_adjusted, self.yaw_adjusted = False, False
        self.yaw, self.distance, self.yaw_delta_t, self.distance_delta_t = None, None, None, None

        self.vel_cmd = Twist()
        self.vel_cmd.linear.y = 0.0
        self.vel_cmd.linear.z = 0.0
        self.vel_cmd.angular.x = 0.0
        self.vel_cmd.angular.y = 0.0

        #---------------- Adjust Parameters ---------------
        self.transformation_x =  0.135
        self.lower_x_threshold = 0.18 - self.transformation_x
        self.upper_x_threshold = 0.26 - self.transformation_x
        self.yaw_threshold = 45*pi/180
        self.linear_velocity = 0.08/2
        self.angular_velocity = 0.35/1
        self.time_compensation = 50*(10**-3) # [s]
        self.yaw_time_scale = 0.1
        #---------------------------------------------------

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.node.create_subscription(PoseArray, '/arm_camera/points',  self.get_next_goal_arm_cam_callback, 10 ) 
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            

    def update(self):
        # Note: hope the behavior tick is slower than the subscriber callback, might need changes
        if not self.position_checked:
            self.yaw = atan2(-self.X_obj + self.transformation_x, -self.Y_obj)
            self.get_logger().info(f"Longitudinal distance = {-self.Y_obj} and Yaw = {self.yaw}")
            self.yaw_delta_t = self.yaw_time_scale *abs(self.yaw)/self.angular_velocity
            self.distance_delta_t =  min(abs(-self.Y_obj- self.lower_x_threshold), abs(-self.Y_obj - self.upper_x_threshold) )/self.linear_velocity 
            self.get_logger().info(f"Distance time = {self.distance_delta_t} [s] and Yaw time = {self.yaw_delta_t} [s]")
            self.position_checked = True
        
        if -self.Y_obj < self.lower_x_threshold or abs(self.yaw) > self.yaw_threshold: #we are too close in the longitudinal direction
            if not self.distance_adjusted:
                self.distance_adjusted = self.adjust_distance()
                return py_trees.common.Status.RUNNING 
            else:
                self.yaw_adjusted = self.adjust_yaw()
                if self.yaw_adjusted:
                    self.distance_adjusted, self.yaw_adjusted, self.position_checked, self.yaw_msg_sent, self.distance_msg_sent = False, False, False, False, False
                    self.yaw, self.distance, self.yaw_delta_t, self.distance_delta_t = None, None, None, None
                    return py_trees.common.Status.FAILURE #return failure to repeat tuck, detect and IK
                else:
                    return py_trees.common.Status.RUNNING
                
        elif -self.Y_obj > self.upper_x_threshold or abs(self.yaw) > self.yaw_threshold: # we are too far in the longitudinal direction
            if not self.yaw_adjusted:
                self.yaw_adjusted = self.adjust_yaw()
                return py_trees.common.Status.RUNNING 
            else:
                self.distance_adjusted = self.adjust_distance()
                if self.distance_adjusted:
                    self.distance_adjusted, self.yaw_adjusted, self.position_checked, self.yaw_msg_sent, self.distance_msg_sent = False, False, False, False, False
                    self.yaw, self.distance, self.yaw_delta_t, self.distance_delta_t = None, None, None, None
                    return py_trees.common.Status.FAILURE #return failure to repeat tuck, detect and IK
                else:
                    return py_trees.common.Status.RUNNING
        else:  # sweet spot
            self.distance_adjusted, self.yaw_adjusted, self.position_checked, self.yaw_msg_sent, self.distance_msg_sent = False, False, False, False, False
            self.yaw, self.distance, self.yaw_delta_t, self.distance_delta_t = None, None, None, None
            return py_trees.common.Status.SUCCESS

    def adjust_distance(self):
        self.vel_cmd.linear.x = self.linear_velocity * (-self.Y_obj - self.lower_x_threshold) /abs((-self.Y_obj - self.lower_x_threshold))
        if not self.distance_msg_sent:
            self.init_time = self.get_clock().now().nanoseconds / 1e9
            self.cmd_vel_publisher.publish(self.vel_cmd)
            self.distance_msg_sent = True
            self.node.get_logger().info(f"Sending adjust distance message: velocity = {self.vel_cmd.linear.x}")
            return False
        else:
            elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.init_time
            if self.get_clock().now().nanoseconds / 1e9 - self.init_time < self.distance_delta_t:
                self.cmd_vel_publisher.publish(self.vel_cmd)
                self.node.get_logger().info(f"Adjusting distance: desired time = {self.yaw_delta_t} and elapsed time = {elapsed_time}")
                return False
            else:
                self.vel_cmd.angular.x = 0.0
                self.cmd_vel_publisher.publish(self.vel_cmd)
                self.node.get_logger().info(f"Distance adjusted")
                return True

    def adjust_yaw(self):
        self.vel_cmd.linear.z = self.angular_velocity
        if not self.yaw_msg_sent:
            self.init_time = self.get_clock().now().nanoseconds / 1e9
            self.cmd_vel_publisher.publish(self.vel_cmd)
            self.yaw_msg_sent = True
            self.node.get_logger().info(f"Sending adjust yaw message: closest object is {self.min}")
            return False
        else:
            elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.init_time
            if elapsed_time < self.yaw_delta_t:
                self.cmd_vel_publisher.publish(self.vel_cmd)
                self.node.get_logger().info(f"Adjusting yaw: desired time = {self.yaw_delta_t} and elapsed time = {elapsed_time}")
                return False
            else:
                self.vel_cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(self.vel_cmd)
                self.node.get_logger().info(f"Yaw adjusted")
                return True

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
            #self.node.get_logger().info(f"Closest object is at {min} [m]: ")
    
