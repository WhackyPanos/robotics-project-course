#!/usr/bin/env python

import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension, Bool, String, Int16
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from handle_objects.ik_solver import IKNode
import PyKDL as kdl
from robp_interfaces.msg import BoxPosition, ObjectPosition
from robp_interfaces.srv import BoxPositionSrv, ObjectPositionSrv
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, PoseArray, Pose2D
from math import pi, acos, atan2, atan, cos, sin, sqrt
import numpy as np 


# ----------------------------------- BEHAVIOUR 1 ---------------------------------------------------------------
class SetArm(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name, angles:list, threshold):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        self.angles = angles
        self.angle_threshold = threshold #150
    
    def setup(self, **kwargs):
            """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
            print("Setting up ObjTuckArm node.")
            self.node = kwargs['node']
            self.arm_started = False
            self.arm_moving = False
            self.arm_tucked = False
            self.done = False
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,  # Ensures message delivery
                durability=DurabilityPolicy.VOLATILE,    # No history saved for late subscribers
                depth=100  # Stores up to 10 messages before dropping old ones
                )
            
            self.servo_angles_subscriber_ = self.node.create_subscription(
                JointState,
                '/servo_pos_publisher',
                self.servo_angles_callback,
                10
            )  


            self.ota_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = qos_profile) # ota = object_tuck_arm
            
            self.obj_tuck_arm_time = 2000 # in ms            
             

            self.desired_servo_angles = [12000]*6
            self.desired_servo_angles[0] = 10000 # gripper is different
            
            # hard coded angles
            self.desired_servo_angles[5] = self.angles[5]
            self.desired_servo_angles[4] = self.angles[4]  
            self.desired_servo_angles[3] = self.angles[3] 
            self.desired_servo_angles[2] = self.angles[2]  
            self.desired_servo_angles[1] = self.angles[1] 
            self.desired_servo_angles[0] = self.angles[0] 


    def servo_angles_callback(self, msg):
        current_angles = msg.position
        if self.arm_started and self.arm_moving:
            self.arm_tucked = True
            self.arm_moving = False
            for i in range(1, len(current_angles)) :
                if abs(self.desired_servo_angles[i] -current_angles[i]) > self.angle_threshold:
                    print(f"Arm still moving (hard-coded movement), error of {abs(self.desired_servo_angles[i] -current_angles[i])}")
                    self.arm_tucked = False
                    self.arm_moving = True
                    break
        #print(f'Current angles position: {current_angles[1]}')

         
    def publish_msg(self):
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
            data_offset=0
        )      
        times = [self.obj_tuck_arm_time] * 6
        msg.data = self.desired_servo_angles + times
        self.ota_publisher_.publish(msg)

    def initialise(self):
        self.arm_started = False
        self.arm_moving = False
        self.arm_tucked = False
        self.done = False
        self.timer = self.node.create_timer(3, self.timer_callback)
        return py_trees.common.Status.RUNNING
        
    def timer_callback(self):
        self.arm_started = True
        self.timer.cancel()
        print(f"delay completed")
         
    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """

        #self.node.get_logger().warn(f"Arm movement: started = {self.arm_started}, moving = {self.arm_moving}, tucked = {self.arm_tucked}")
        if self.done:
            return py_trees.common.Status.SUCCESS
        elif not self.arm_started and not self.arm_tucked and not self.arm_moving: #initial condition, before the delay
            return py_trees.common.Status.RUNNING
        
        elif self.arm_started and not self.arm_moving and not self.arm_tucked: #after the timer callback
            self.publish_msg()
            self.arm_moving = True
            return py_trees.common.Status.RUNNING  # Keep running while the arm moves
        
        elif self.arm_started and self.arm_moving and not self.arm_tucked:
            #self.publish_msg()
            return py_trees.common.Status.RUNNING # Keep running while the arm moves
        
        elif self.arm_tucked:
            self.arm_started,self.arm_tucked, self.arm_moving, self.done= False, False, False, True
            return py_trees.common.Status.SUCCESS

        else:
             print("ERROR")
             
    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        if new_status == py_trees.common.Status.SUCCESS:
            self.arm_tucked == True # not really necessary
            #print(f"New status is {new_status}")


    def __init__(self, name="Detect Object"):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        #py_trees.behaviour.Behaviour.__init__(self, name)
        #Node.__init__(self, "pick_node")  # ROS 2 node initialization
        self.ik_solver = IKNode()

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']
        
        
    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""    
         
    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
# ----------------------------------- BEHAVIOUR 3 ---------------------------------------------------------------
class SearchObjectArm(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Detect Object"):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node



    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']
        
        
    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""    
         
    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

# ----------------------------------- BEHAVIOUR 4---------------------------------------------------------------
class ArmIK(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="InverseKin"):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node

        self.ik_solver = IKNode()
        self.to_frame_rel = 'arm_base_link'
        self.from_frame_rel = 'map'
        self.X, self.Y,  self.x, self.y, self.z, self.target = None, None, None, None, None, None
        self.X_arm_cam, self.Y_arm_cam = [],  []
        self.thresholds = [10**-4,10**-3,10**-2]
        self.initial_guesses = [[0,0,0,0,0,0]]
        self.current_angles, self.desired_servo_angles = None, None
        self.fail_count = 0
        # joint limits in the arm domain
        self.lb_angles = [0.0, 0.0,  30.0, 30.0, 60.0, 0.0]
        self.ub_angles = [90.0,240.0,210.0,210.0,180.0,240.0]
        # joint limits in normal domain
        self.lb_q = [-120.0,-90.0,-105.0,-105.0,-120.0, 0.0] # original: [-120.0,-60.0,-90.0,-90.0,-120.0,0.0]
        self.ub_q = [ 120.0, 90.0, 105.0, 105.0, 120.0, 0.0] #original: [120.0,60.0,90.0,90.0,120.0,0.0]

        # ------------------------------- Wrist Parameters ------------------------------------------
        self.max_rotation = 90
        self.angle_step= 30 
        # -------------------------------------------------------------------------------------------

        # ----------------------------- Gripper Parameters ------------------------------------------
        # -------------------------------------------------------------------------------------------
        """ max gripper angle with object = 9850, 9624 (plushy)
            max gripper angle without object = 9950 """
        self.angle_threshold = 190 # threshold for joint angles in IK
        self.closed_gripper_angle = 10000 # gripper angle to grasp object
        self.gripper_threshold = 500 # threshold to detect if something was grasped
        self.open_gripper_angle = 1000 # gripper angle when reaching for object
        self.time_to_wait_for_gripper_angle = 2 # [s]
        self.init_time_check_gripper_angle = None
        self.curr_time_check_gripper_angle = None
        self.joint3_compensation = -2500 # compemsation for joint 3 (below wrist). was -2000 in MS3 video
        self.skipped_solutions = 1

        self.obj_tuck_arm_time = 3000 # in ms
        self.obj_grasp_time = 2000  

        # ----------------------- Inverse kinematics parameters -------------------------------------
        # -------------------------------------------------------------------------------------------
        self.height = 93*(10**-3)
        self.l0 = 101*(10**-3) #101
        self.l1 = 95*(10**-3) # 95
        self.l2 = 158*(10**-3)+ 0.0  #1cm to account for object height and avoid colliding with ground  168
        self.xx = 0.137 + 0.02 # compensation for the joint 3 compensation
        self.yy = 0.0 # positive is to left in robot perspective
        self.zz = -0.06
        # -------------------------------------------------------------------------------------------
        # -------------------------------------------------------------------------------------------


    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']
        self.dopmc_available = False # dopac = detected object position in the arm camera (frame)
        self.ready2move= False
        self.arm_moving = False
        self.arm_tucked = False  
        self.object_grasped = False  
        self.done = False
        

        # Initialize the transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.servo_angles_subscriber_ = self.node.create_subscription(JointState,'/servo_pos_publisher',self.servo_angles_callback,10)   
        self.next_goal_pub = self.node.create_subscription(PoseStamped, '/motion/goal',  self.get_next_goal_callback,10 )
        self.next_goal_pub = self.node.create_subscription(PoseArray, '/arm_camera/points',  self.get_next_goal_arm_cam_callback, 10 )
        self.count_grasping_failures_sub = self.node.create_subscriber(
            Int16,'/picklift/count_grasping_failures',  self.count_grasping_failures_callback,10)

        self.ota_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = 10) # ota = object_tuck_arm
        self.picklift_pub = self.node.create_publisher(Bool, '/picklift/succeded', 10)
              
        
    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""
        self.done = False #TODO: review
        self.ready2move = False 
        self.arm_moving = False
        self.arm_tucked = False
        self.object_grasped = False
        self.X, self.Y,  self.x, self.y, self.z, self.target = None, None, None, None, None, None
        self.current_angles, self.desired_servo_angles = None, None
        self.timer = self.node.create_timer(1, self.init_arm_movement)

         
    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        #self.node.get_logger().info(f"Objects in arm camera frame {[self.X_arm_cam, self.Y_arm_cam]}")
        if self.done:
            return py_trees.common.Status.SUCCESS
        elif self.ready2move and not self.arm_moving and not self.arm_tucked : # if necessary, self.x is not None and self.y is not None and self.z is not None and 
            # if everything fails, try a simpler approach. Transform from arm camera to arm base
            msg = self.pick_planB(-self.Y_arm_cam[0] + self.xx , -self.X_arm_cam[0] + self.yy, self.zz)
            self.node.get_logger().info(f"Trying to reach {[-self.Y_arm_cam[0] + self.xx, -self.X_arm_cam[0] + 0.0, self.zz]}")
            if msg is not None:
                self.ota_publisher_.publish(msg)
                #self.move_timer = self.node.create_timer(self.obj_tuck_arm_time/1000 + 3.0, self.wait_for_movement)
                self.arm_moving = True
                self.node.get_logger().warn(f"Message published, arm is moving")
                return py_trees.common.Status.RUNNING
            else:
                # try new objects in the list. If it is empty, it failed
                self.X_arm_cam.pop(0)
                self.Y_arm_cam.pop(0)

                if len(self.X_arm_cam) == 0:
                    #self.picklift_pub.publish(Bool(data=False))  # TODO: if pick and search fails, a message has to be published. That can happen here or in the arm camera
                    self.node.get_logger().warn(f"IK FAILED")
                    self.picklift_pub.publish(Bool(data=False))
                    return py_trees.common.Status.FAILURE
                else:
                    self.arm_moving = False
                    msg = None
                    self.node.get_logger().warn(f"CHOOSING ANOTHER OBJEC")
                    return py_trees.common.Status.RUNNING


        # if arm is moving but not in grasp position, return keep running
        elif self.arm_moving and not self.arm_tucked:
            #self.node.get_logger().warn(f"Arm is still  moving")
            return py_trees.common.Status.RUNNING 

        # if arm is in grasp position,  start grasping 
        elif self.arm_tucked and not self.object_grasped and not self.arm_moving:
            #self.node.get_logger().info(f"Arm Moving = {self.arm_moving}, Arm Tucked = {self.arm_tucked} and grasped = {self.object_grasped}")
            msg = Int16MultiArray()
            msg.layout = MultiArrayLayout(
                dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
                data_offset=0)  
            times = [self.obj_grasp_time] * 6
            self.desired_servo_angles[0] = self.closed_gripper_angle 
            msg.data = self.desired_servo_angles + times
            self.ota_publisher_.publish(msg)
            self.move_timer = self.node.create_timer((self.obj_grasp_time/1000) + 4 , self.wait_for_movement)
            self.arm_moving = True
            return py_trees.common.Status.RUNNING

        elif self.arm_tucked and not self.object_grasped and self.arm_moving:
            return py_trees.common.Status.RUNNING 

        # if object is grasped, return success
        elif self.object_grasped:
            self.curr_time_check_gripper_angle = self.get_clock().now().nanoseconds / 1e9
            if self.curr_time_check_gripper_angle - self.init_time_check_gripper_angle < self.time_to_wait_for_gripper_angle:
                return py_trees.common.Status.RUNNING 
            else:
                self.node.get_logger().warn(f"Timer finished, dt = {self.curr_time_check_gripper_angle - self.init_time_check_gripper_angle}, checking gripper")
                if abs(self.desired_servo_angles[0] -self.current_angles[0]) < self.gripper_threshold:
                    self.picklift_pub.publish(Bool(data=False))
                    self.node.get_logger().warn(f"NOTHING GRASPED, desired angle = {self.desired_servo_angles[0]} and angle = {self.current_angles[0]}")
                else:
                    self.picklift_pub.publish(Bool(data=True))
                    self.node.get_logger().warn(f"SOMETHING GRASPED, desired angle = {self.desired_servo_angles[0]} and angle = {self.current_angles[0]}")

                self.node.get_logger().info(f"Lifting arm now")
                self.done = True
                self.X_arm_cam.pop(0)
                self.Y_arm_cam.pop(0)
                return py_trees.common.Status.SUCCESS

        else:
            return py_trees.common.Status.RUNNING  
            
    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        if self.object_grasped:
            return py_trees.common.Status.SUCCESS

    def init_arm_movement(self):
        """ trigger flat to start arm movement"""
        self.ready2move = True
        self.timer.cancel()

    def wait_for_movement(self):
        self.object_grasped = True
        self.arm_moving = False
        self.init_time_check_gripper_angle = self.get_clock().now().nanoseconds / 1e9
        self.move_timer.cancel()
        
    def count_grasping_failures_callback(self, msg):
        self.fail_count = msg.data

    def adjust_retry(self):
        if self.fail_count != 0:
            wrist_adjustments_count = 0
            # adjust wrist rotation "randomly"
            while True:
                self.desired_servo_angles[1] += self.angle_step*100
                if not(self.desired_servo_angles[1] < self.lb_q[1] or self.desired_servo_angles[1] > self.ub_q[1]):
                    break
                else:
                    self.desired_servo_angles[1] = 12000
                    self.angle_step *= -1
                wrist_adjustments_count += 1
                if wrist_adjustments_count > 10:
                    self.desired_servo_angles[1] = 12000
                    break
        # adjust rotation of the 3 servo (below wrist)
        #self.desired_servo_angles[2] =


    def get_next_goal_callback(self, msg):
        self.X = msg.pose.position.x
        self.Y = msg.pose.position.y

    def get_next_goal_arm_cam_callback(self, msg):
        if len(self.X_arm_cam) != 0:
            self.X_arm_cam, self.Y_arm_cam = [],  []
        for pose in msg.poses:
            self.X_arm_cam.append(pose.position.x)
            self.Y_arm_cam.append(pose.position.y)


    def servo_angles_callback(self, msg):
        """ Callback to check if arm is in a good enough position while moving. improve later on"""
        #self.node.get_logger().warn(f"Checking joints")
        self.current_angles = msg.position
        if self.arm_moving and not self.arm_tucked and not self.object_grasped:
            self.arm_tucked = True
            self.arm_moving = False
            for i in range(1, len(self.current_angles)) :
                if abs(self.desired_servo_angles[i] -self.current_angles[i]) > self.angle_threshold:
                    #self.node.get_logger().info(f"Arm still moving, error of {abs(self.desired_servo_angles[i] -self.current_angles[i])} in joint {i+1}")
                    self.arm_tucked = False
                    self.arm_moving = True
                    break
        
    def pick_planB(self, x, y, z):  

        # we will iterate with different orientations for the 1st link. 
        count = 0
        for i in range(900, 0, -1): #decidegrees. Before, was range(900,0,-20)
            angle = i/10
            # Initiate constants
            good_flag = True           
            q = [0.0, 0.0, 0.0, 0.0] #joint angles, from base to last y joint, respectively

            # Account for the geometry (different frame)
            new_z = -(abs(z) + self.height+ self.l0*sin(angle*pi/180))
            new_x = sqrt(x**2 + y**2)-self.l0*cos(angle*pi/180)
            theta = atan2(y,x) #orientation of the arm base

            # Compute & assign angles
            k = (new_x)**2 + new_z**2 - self.l1**2 - self.l2**2 # Auxiliary constant
            try:
                q2 = acos(k/(2*self.l1*self.l2)) # acos returns between 0 and pi
            except:
               #self.node.get_logger().info(f" x = {new_x} and z = {new_z} not reacheable ({abs(new_z), self.a, self.b, l0*sin(i*pi/180)})")
                continue           
            q[0] = theta*180/pi
            q[1] = 90 - angle
            q[2] = (atan2(new_z, new_x) - atan2(self.l2*sin(q2), self.l1 + self.l2*cos(q2)))*180/pi
            q[3] = q2*180/pi

            # check for limits and end loop if everything is ok
            for j in range(len(q)):
                if q[j] < self.lb_q[j] or q[j] > self.ub_q[j]:
                    good_flag = False
                    #self.node.get_logger().info(f"Joint {j} angle is outside limits (angle = {q[j+1]})")
            if good_flag == True:
                if count == self.skipped_solutions -1: # skip  to avoid joint limits
                    count +=1
                    continue
                # create the msg to publish, in case it is valid
                self.node.get_logger().info(f"Plan B worked: in the regular domain, angles = {q}")

                msg = Int16MultiArray()
                msg.layout = MultiArrayLayout(
                    dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
                    data_offset=0
                )      
                times = [self.obj_tuck_arm_time] * 6

                # tranform from regular domain to arm domain
                self.desired_servo_angles = [12000]*6
                self.desired_servo_angles[0] = self.open_gripper_angle

                # update with plan_b
                self.desired_servo_angles[5] = self.desired_servo_angles[5] + int(100*q[0])   
                self.desired_servo_angles[4] = self.desired_servo_angles[4] + int(-100*q[1])  
                self.desired_servo_angles[3] = self.desired_servo_angles[3] + int(-100*q[2])
                self.desired_servo_angles[2] = self.desired_servo_angles[2] + int(-100*q[3]) + self.joint3_compensation 
                self.adjust_retry()        
                msg.data = self.desired_servo_angles + times
                return msg
            
        # if no solution was found, msg is invalid
        msg = None
        self.node.get_logger().error("Plan B did not work, returning FAILURE")
        return msg


class Place(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Place"):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        #self.logger.debug("ObjTuckArm was called.")
        #self.cached_context = None
        self.next_goal = 'Object'
    
    def setup(self, **kwargs):
            """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
            print("Setting up ObjTuckArm node.")
            self.node = kwargs['node']
            self.arm_started = False
            self.gripper_opened = False
            self.gripper_is_opening = False
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,  # Ensures message delivery
                durability=DurabilityPolicy.VOLATILE,    # No history saved for late subscribers
                depth=100  # Stores up to 10 messages before dropping old ones
                )
            
            self.servo_angles_subscriber_ = self.node.create_subscription(
                JointState,
                '/servo_pos_publisher',
                self.servo_angles_callback,
                10)  
            self.need_next_object_sub = self.node.create_subscription(
                String, '/next_goal/object/need', 
                self.need_next_object_callback, 10)
            self._pose_pub = self.create_publisher(
                Pose2D, '/odom_pose', self.odometry_yaw_callback, 10)
            self.create_subscription(
                PoseStamped, '/motion/goal', self.goal_callback, 10)
            
            self.ota_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = qos_profile) # ota = object_tuck_arm
            self.picklift_pub = self.node.create_publisher(Bool, '/picklift/succeded', 10)

            self.obj_tuck_arm_time = 2000 # in ms            
             
            # init tuck arm angles
            #self.desired_servo_angles = [45, 230, 80.5232360099144, 201.20685353937313, 68.846193182243155, 139.65382600499612]
            self.desired_servo_angles = [12000]*6
            self.desired_servo_angles[0] = 10000 # gripper is different
            self.desired_servo_angles[4] = 6500 # 
            self.desired_servo_angles[2] = 8000 # 
            self.angle_threshold = 200 #1 degree 

            # stuff related to the orientation of the arm base
            self.current_yaw = 0 
            self.box_map = PointStamped()
            self.box_map.header.frame_id = 'map'
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer)
            self.box_x, self.box_y = None, None


    def initialise(self):
        self.arm_moving = False
        self.arm_tucked = False
        self.timer = self.node.create_timer(3, self.timer_callback)
        
        
    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked 
            If the next goal is to place an object, the arm will be tucked. Otherwise, we want to tick the picking behavior"""
        if self.next_goal == 'Object':
            return py_trees.common.Status.FAILURE
        else:
            #self.get_logger().info(f"Placing: started = {self.arm_started}, moving = {self.arm_moving}, tucked = {self.arm_tucked}")

            if not self.arm_started and not self.arm_tucked and not self.arm_moving: #initial condition, before the delay
                return py_trees.common.Status.RUNNING
            
            elif self.arm_started and not self.arm_moving and not self.arm_tucked: #after the timer callback
                self.publish_msg()
                #self.get_logger().info("Publishing INITIAL tuck arm command.")
                self.arm_moving = True
                return py_trees.common.Status.RUNNING  # Keep running while the arm moves
            
            elif self.arm_started and self.arm_moving and not self.arm_tucked:
                #self.publish_msg()
                #self.get_logger().info("Arm moving.")
                return py_trees.common.Status.RUNNING # Keep running while the arm moves
            
            elif self.arm_tucked:
                if not self.gripper_is_opening and not self.gripper_opened:
                    #self.get_logger().info("Arm is in a good position")
                    self.desired_servo_angles[0] = 2600
                    self.publish_msg()
                    self.open_gripper_timer = self.node.create_timer(4.0, self.open_gripper_callback)
                    self.gripper_is_opening = True
                    return py_trees.common.Status.RUNNING
                elif self.gripper_is_opening and not self.gripper_opened:
                    return py_trees.common.Status.RUNNING
                else:
                    self.arm_started,self.arm_tucked, self.arm_moving, self.gripper_opened, self.gripper_is_opening = False, False, False, False, False
                    self.picklift_pub.publish(Bool(data=True))
                    return py_trees.common.Status.SUCCESS

            else:
                print("ERROR")
        return
             
    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        if new_status == py_trees.common.Status.SUCCESS:
            self.arm_tucked == True # not really necessary
            #print(f"New status is {new_status}")

    def timer_callback(self):
        self.arm_started = True
        self.timer.cancel()
        print(f"delay completed")

    def odometry_yaw_callback(self, msg):
        self.current_yaw = msg.theta

    def goal_callback(self, msg):
        self.box_x = msg.pose.position.x
        self.box_y = msg.pose.position.y

    def desired_arm_base_angle(self):
        self.box_map.header.stamp = rclpy.time.Time().to_msg()
        self.box_map.header.frame_id = 'map'
        self.box_map.point.x = self.box_x
        self.box_map.point.y = self.box_y
        self.box_arm_base = self.tf_buffer.transform(self.box_map, 'arm_base_link', timeout=rclpy.duration.Duration(seconds=1.0))

        return atan2(self.box_arm_base.point.y, self.box_arm_base.point.x) # TODO: check signal

    def open_gripper_callback(self):
        self.gripper_opened = True

    def servo_angles_callback(self, msg):
        current_angles = msg.position
        if self.arm_started and self.arm_moving:
            self.arm_tucked = True
            self.arm_moving = False
            for i in range(1, len(current_angles)) :
                if abs(self.desired_servo_angles[i] -current_angles[i]) > self.angle_threshold:
                    #print(f"Arm still moving, error of {abs(self.desired_servo_angles[i] -current_angles[i])}")
                    self.arm_tucked = False
                    self.arm_moving = True
                    break
        #print(f'Current angles position: {current_angles[1]}')
         
    def need_next_object_callback(self, msg):
        self.next_goal = msg.data
        self.get_logger().info(f"Placing received {msg.data} as next stuff")
    def publish_msg(self):
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
            data_offset=0)    
          
        # Compute necessary orientation of arm base
        desired_yaw = self.desired_arm_base_angle()
        error = 180*(desired_yaw - self.current_yaw)/pi #TODO: wrap angles, check limits and uncomment below
        #self.desired_servo_angles[5] += error*100
        self.get_logger().info(f'Arm base should rotate {error} degrees')

        # publish msg
        times = [self.obj_tuck_arm_time] * 6
        msg.data = self.desired_servo_angles + times
        self.ota_publisher_.publish(msg)
 # -----------------------------------------------------------------------------------------------------------------------


""""
IK

                self.object_transform() # tranform object position from map frame to arm base frame
                # target_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(self.x, self.y, self.z))
                # for j,IG in enumerate(self.initial_guesses):
                #     for i, thresh in enumerate(self.thresholds):
                #         result, angles = self.ik_solver.solve_ik(target_pose, IG, thresh, 100000)
                #         if result >= 0:
                #             # publish corrected angles in the servo_pos topic. this function already shows message in terminal
                #             out_limits = self.publish_angles(angles)
                #             if not out_limits:
                #                 self.arm_moving = True
                #                 return py_trees.common.Status.RUNNING
                #             else:
                #                 break
                #         else:
                #             self.node.get_logger().info(f"IK Solver failed for {thresh}, trying bigger error threshold!")
                #     self.node.get_logger().info(f"IK Solver failed for all thresholds, trying different initial guess!")
                self.node.get_logger().warn(f"COULD NOT GET SOLUTION, trying plan B")"""


    # def object_transform(self):
    #     # get transform from map frame to frame of the arm base
    #     time = rclpy.time.Time() #retrieve most recent transform ig
    #     try:
    #         t = self.tf_buffer.lookup_transform(
    #             self.to_frame_rel,
    #             self.from_frame_rel,
    #             time)
    #     except TransformException as ex:
    #         self.node.get_logger().info(
    #             f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}')
    #         return
        
    #     # do transformation   
    #     object_MF = PointStamped()
    #     object_MF.header.frame_id = 'map'
    #     object_MF.point.x = self.X
    #     object_MF.point.y = self.Y
    #     object_MF.point.z = 0.0         
    #     object_ABF = tf2_geometry_msgs.do_transform_point(object_MF, t)
    #     self.x = 0.20# TODO: update later object_ABF.point.x
    #     self.y = 0.05# TODO: object_ABF.point.y
    #     self.z = object_ABF.point.z
        
    #     # define target with kdl instance
    #     self.node.get_logger().info(f'Object ({self.X, self.Y}) in map -> ({self.x,self.y,self.z}) in arm_base')

    # def publish_angles(self, ik_angles):
    #     """ Publish corrected angles to the arm after having computing inverse kinematics"""
    #     # retrieve current angles (supposedly arm is stretched) and compute corrected angles to publish
    #     init_angles = np.multiply(0.01,np.array(self.current_angles)) # from servo sensors, pass from cdeg to deg
    #     desired_angles = self.wrap_angle((180/pi)*np.array(ik_angles[::-1]), "degrees") # convert ik_angles to angles bw -180 and 180        
    #     res_angles = init_angles + np.multiply(ik_angles[::-1], [1,1,-1,1,-1,1]) # CHECK THIS VECTOR AGAIN
    #     pub_angles, out_limits = self.check_limits(res_angles, self.lb_angles, self.ub_angles)

    #     self.node.get_logger().info(f"Current angles are {init_angles}") 
    #     self.node.get_logger().info(f"IK angles after wrapping are {desired_angles}")
    #     self.node.get_logger().info(f"Resulting angles {res_angles}")

    #     msg = Int16MultiArray()
    #     msg.layout = MultiArrayLayout(
    #         dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
    #         data_offset=0
    #     )      
    #     times = [self.obj_tuck_arm_time] * 6
    #     self.desired_servo_angles = [int(angle*100) for angle in pub_angles]
    #     msg.data = self.desired_servo_angles + times
    #     msg.data[0] = 2600 #keep gripper open
        

    #     if not out_limits:
    #         self.node.get_logger().info(f" Angles will be published with ik_solver = {self.desired_servo_angles}")
    #         self.ota_publisher_.publish(msg)
    #         #self.move_timer = self.node.create_timer(self.obj_tuck_arm_time/1000 + 3.0, self.wait_for_movement)
    #     else:
    #         self.node.get_logger().info(f" Angles out of limits, trying again ")
            
    #     return out_limits

    # def wrap_angle(self, angles, units: str):
    #     if units == "radians":
    #         return [(angle + pi) % (2 * pi) - pi for angle in angles]
    #     else:
    #         return [(angle + 180) % 360 - 180 for angle in angles]
        
    # def check_limits(self, angles, lb, ub):
    #     final_angles = angles
    #     out_limits = False
    #     for i,x in enumerate(angles):
    #         if x<lb[i]:
    #             final_angles[i] = lb[i]+1
    #             out_limits = True
    #         elif x>ub[i]:
    #             final_angles[i] = ub[i]-1
    #             out_limits = True
    #     return final_angles, out_limits