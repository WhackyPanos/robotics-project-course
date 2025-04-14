#!/usr/bin/env python

import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension, Bool, String, Int16
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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
    def __init__(self, name, angles, threshold):
        #super().__init__(self, name = name)
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        self.angles = angles
        self.angle_threshold = threshold #150
        self.blackboard = py_trees.blackboard.Blackboard()
    
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

            """ === Panos: added goal publisher for adjusting yaw """
            self.motion_goal_publisher_ = self.node.create_publisher(
                PoseStamped,
                '/motion/goal',
                10
            )
            """ === """

            self.ota_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = qos_profile) # ota = object_tuck_arm
            self.picklift_pub = self.node.create_publisher(Bool, '/picklift/succeded', 10)
            self.trigger_mapping = self.node.create_publisher(Bool, '/do_mapping', 10)
            
            self.obj_tuck_arm_time = 2000 # in ms            

            self.current_angles = None
            self.gripper_threshold = 420 # threshold to detect if something was grasped
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
        self.current_angles = msg.position
        if self.arm_started and self.arm_moving:
            self.arm_tucked = True
            self.arm_moving = False
            for i in range(1, len(self.current_angles)) :
                if abs(self.desired_servo_angles[i] - self.current_angles[i]) > self.angle_threshold:
                    #print(f"Arm still moving (hard-coded movement), error of {abs(self.desired_servo_angles[i] - self.current_angles[i])}")
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
        self.arm_tucked_timer_counts = 0
        self.done = False
        self.timer = self.node.create_timer(3, self.timer_callback)
        self.get_logger().info("Tuck arm behavior was initialized")
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
            # self.publish_msg()
            return py_trees.common.Status.RUNNING # Keep running while the arm moves
        
        elif self.arm_tucked and self.arm_tucked_timer_counts <= 10:
            self.arm_tucked_timer_counts += 1
            return py_trees.common.Status.RUNNING

        elif self.arm_tucked and self.arm_tucked_timer_counts > 10:
            self.arm_started,self.arm_tucked, self.arm_moving, self.done= False, False, False, True
            if self.name == 'lift':
                if abs(self.desired_servo_angles[0] -self.current_angles[0]) < self.gripper_threshold:
                    self.picklift_pub.publish(Bool(data=False))
                    self.node.get_logger().warn(f"NOTHING GRASPED, desired angle = {self.desired_servo_angles[0]} and angle = {self.current_angles[0]}")
                    self.blackboard.pick_status = py_trees.common.Status.FAILURE
                    return py_trees.common.Status.FAILURE # HACK: trying to get decorator working
                else:
                    self.picklift_pub.publish(Bool(data=True))
                    self.node.get_logger().warn(f"SOMETHING GRASPED, desired angle = {self.desired_servo_angles[0]} and angle = {self.current_angles[0]}")
                    self.trigger_mapping.publish(Bool(data = True))
                    self.blackboard.pick_status = py_trees.common.Status.SUCCESS
                    return py_trees.common.Status.SUCCESS
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
        self.X, self.Y = None, None
        self.X_planB, self.Y_planB = None, None
        self.X_arm_cam, self.Y_arm_cam, self.wrist_angle = [],  [], []
        self.thresholds = [10**-4,10**-3,10**-2]
        self.initial_guesses = [[0,0,0,0,0,0]]
        self.current_angles, self.desired_servo_angles = None, None
        self.fail_count = 0

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
        self.open_gripper_angle = 1000 # gripper angle when reaching for object
        self.time_to_wait_for_gripper_angle = 0.1 # [s]
        self.init_time_check_gripper_angle = None
        self.curr_time_check_gripper_angle = None
        self.joint3_compensation = 0 # compemsation for joint 3 (below wrist). was -2000 in MS3 video
        self.skipped_solutions = 1

        self.obj_tuck_arm_time = 3000 # in ms
        self.obj_grasp_time = 2000  

        # ----------------------- Inverse kinematics parameters -------------------------------------
        # -------------------------------------------------------------------------------------------
        self.height = 93*(10**-3)
        self.l0 = 101*(10**-3) #101
        self.l1 = 94*(10**-3) # 95
        self.l2 = 162*(10**-3)+ 0.0  #1cm to account for object height and avoid colliding with ground  168

        self.xx = 0.135 # compensation for the joint 3 compensation
        self.yy = 0.0 # positive is to left in robot perspective
        self.zz = -0.055 # TODO: this is the variable to change if the robot is too far or too close to the ground

        # angles in decidegrees
        self.joint_5_start_angle = 50 
        self.joint_5_end_angle = 900
        self.joint_5_step = 5

        self.X_test = 0.18
        self.Y_test = 0.05

        # joint limits in the arm domain
        # self.lb_angles = [0.0, 0.0,  30.0, 30.0, 60.0, 0.0]
        # self.ub_angles = [90.0,240.0,210.0,210.0,180.0,240.0]

        # joint limits in normal domain
        margin = 1
        self.lb_q = [-120.0,-90.0 + margin ,-90 + margin ,-90 + margin ,-120.0 + margin , 0.0] # original: [-120.0,-60.0,-90.0,-90.0,-120.0,0.0]
        self.ub_q = [ 120.0, 90.0 - margin , 90 - margin , 90 - margin , 120.0 - margin , 0.0] #original: [120.0,60.0,90.0,90.0,120.0,0.0]
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
        self.object_type = None
        

        # Initialize the transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.servo_angles_subscriber_ = self.node.create_subscription(JointState,'/servo_pos_publisher',self.servo_angles_callback,10)   
        self.node.create_subscription(PoseStamped, '/motion/goal',  self.get_next_goal_callback,10 )
        self.node.create_subscription(PoseArray, '/arm_camera/points',  self.get_next_goal_arm_cam_callback, 
                                      rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.count_grasping_failures_sub = self.node.create_subscription(
            Int16,'/picklift/count_grasping_failures',  self.count_grasping_failures_callback,10)
        self.node.create_subscription(String, '/object_type', self.get_object_type_callback, 10)

        self.ota_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = 10) # ota = object_tuck_arm

        self.blackboard = py_trees.blackboard.Blackboard()
        
    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""
        self.done = False #TODO: review
        self.ready2move = False 
        self.arm_moving = False
        self.arm_tucked = False
        self.object_grasped = False
        #self.X, self.Y = None, None
        self.current_angles, self.desired_servo_angles = None, None
        self.timer = self.node.create_timer(1, self.init_arm_movement)


    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        #self.node.get_logger().info(f"Objects in arm camera frame {[self.X_arm_cam, self.Y_arm_cam]}")
        if self.done:
            return py_trees.common.Status.SUCCESS
        elif self.ready2move and not self.arm_moving and not self.arm_tucked : # if necessary, self.x is not None and self.y is not None and self.z is not None and 
            # if everything fails, try a simpler approach. Transform from arm camera to arm base
            
            # TODO: include adjust yaw routine!
            
            if len(self.Y_arm_cam) == 0: #arm_segmentation is shit, use other stuff
                self.object_transform()
                self.node.get_logger().warn(f"Using transform from map to arm_base_link instead")
                self.X_arm_cam.append(-self.Y_planB + self.yy)
                self.Y_arm_cam.append(-self.X_planB + self.xx)
            msg = self.pick_planB(-self.Y_arm_cam[0] + self.xx , -self.X_arm_cam[0] + self.yy, self.zz)
            #self.node.get_logger().info(f"Trying to reach {[-self.Y_arm_cam[0] + self.xx, -self.X_arm_cam[0] + 0.0, self.zz]}")
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
                self.wrist_angle.pop(0)

                if len(self.X_arm_cam) == 0:
                    #self.picklift_pub.publish(Bool(data=False))  # TODO: if pick and search fails, a message has to be published. That can happen here or in the arm camera
                    self.node.get_logger().warn(f"IK FAILED")
                    #self.picklift_pub.publish(Bool(data=False))
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
            self.move_timer = self.node.create_timer((self.obj_grasp_time/1000) + 2 , self.wait_for_movement)
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
                #self.node.get_logger().warn(f"Timer finished, dt = {self.curr_time_check_gripper_angle - self.init_time_check_gripper_angle}")
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

    def adjust_wrist(self):
        wrist_angle = self.wrist_angle[0] * 180/pi
        if self.object_type == "3":
            wrist_angle += 45
        self.get_logger().info(f"Wrist angle is {round(self.wrist_angle[0], 2)} and object type is {self.object_type}")
        wrist_angle = ((wrist_angle + 90) % 180) - 90 #map to [-90, 90]
        wrist_angle = int(100*wrist_angle + 12000)

        return wrist_angle

    def get_object_type_callback(self, msg):
        self.object_type = msg.data

    def get_next_goal_callback(self, msg):
        self.node.get_logger().info(f"IK received object in map frame (in case arm camera does not work) -> {msg.pose.position.x, msg.pose.position.y}")
        self.X = msg.pose.position.x
        self.Y = msg.pose.position.y

    def get_next_goal_arm_cam_callback(self, msg):
        if len(self.X_arm_cam) != 0:
            self.X_arm_cam, self.Y_arm_cam = [],  []
        for pose in msg.poses:
            self.X_arm_cam.append(pose.position.x)
            self.Y_arm_cam.append(pose.position.y)

            # to transformation from quaternions to wrist angle
            qz = pose.orientation.z
            qw = pose.orientation.w
            self.wrist_angle.append(2*atan2(qz, qw))


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
        self.get_logger().info(f" For IK solver, object is in {round(x*100, 2), round(y*100, 2), round(z*100, 2)} in arm_base link")
        # we will iterate with different orientations for the 1st link. 
        count = 0
        for i in range(self.joint_5_start_angle, self.joint_5_end_angle, self.joint_5_step): #decidegrees. Before, was range(900,0,-20)
            angle = i/10
            # Initiate constants
            good_flag = True           
            q = [0.0, 0.0, 0.0, 0.0] #joint angles, from base to last y joint, respectively

            # Account for the geometry (different frame)
            new_z = -(abs(z) + self.height+ self.l0*sin(angle*pi/180))
            new_x = sqrt(x**2 + y**2)-self.l0*cos(angle*pi/180)
            theta = atan2(y,x) #orientation of the arm base

            # Compute & assign angles
            k = new_x**2 + y**2 + new_z**2 - self.l1**2 - self.l2**2 # Auxiliary constant
            c2 = k/(2*self.l1*self.l2)
            try:
                q2 = acos(k/(2*self.l1*self.l2))
            except:
                #print(f" x = {new_x} and z = {new_z} not reacheable")
                continue               
            q[0] = theta*180/pi
            q[1] = 90 - angle
            q[2] = (atan2(new_z, new_x) + atan2(self.l2*sin(q2), self.l1 + self.l2*cos(q2)))*180/pi - angle
            q[3] = q2*180/pi
            if q[2] > 0:
                q[2] -= 180

            # check for limits and end loop if everything is ok
            for j in range(len(q)):
                if q[j] < self.lb_q[j] or q[j] > self.ub_q[j]:
                    good_flag = False
                    self.node.get_logger().info(f"Joint {j} angle is outside limits (angle = {q[j]})")
            if good_flag == True:
                if count == self.skipped_solutions -1: # skip  to avoid joint limits
                    count +=1
                    continue
                # create the msg to publish, in case it is valid
                self.node.get_logger().info(f"Object is reachable: in the regular domain, angles = {q}")                          
                x_calc = self.l0*cos(angle*pi/180) + self.l1*cos((angle + q[2])*pi/180) + self.l2*cos((angle + q[2] - q[3])*pi/180)
                z_calc = self.l0*sin(angle*pi/180) + self.l1*sin((angle + q[2])*pi/180) + self.l2*sin((angle + q[2] - q[3])*pi/180) + self.height
                print(f" Forward dynamics: x = {round(x_calc*100,2)} and z = {round(z_calc*100, 2)}")

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
                self.desired_servo_angles[1] = self.adjust_wrist()        
                msg.data = self.desired_servo_angles + times
                return msg
            
        # if no solution was found, msg is invalid
        msg = None
        self.node.get_logger().error("Plan B did not work, returning FAILURE")
        return msg

    def object_transform(self):
        # get transform from map frame to frame of the arm base
        time = rclpy.time.Time() #retrieve most recent transform ig
        try:
            t = self.tf_buffer.lookup_transform(
                'arm_base_link',
                'map',
                time)
        except TransformException as ex:
            self.node.get_logger().info(
                f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}')
            return
        
        # do transformation
        self.node.get_logger().info(f'Tranform between map and arm_base_link: {t.transform.translation.x,t.transform.translation.y}') 
        object_MF = PointStamped()
        object_MF.header.stamp = t.header.stamp
        object_MF.header.frame_id = 'map'
        object_MF.point.x = self.X
        object_MF.point.y = self.Y
        object_MF.point.z = 0.0           
        object_ABF = tf2_geometry_msgs.do_transform_point(object_MF, t)
        self.X_planB = object_ABF.point.x 
        self.Y_planB = object_ABF.point.y 

        self.node.get_logger().info(f'Using transform between map and arm base, x = {self.X_planB}, y = {self.Y_planB}')

        # TODO: hardcoded now, remove later 
        # self.X_planB = self.X_test
        # self.Y_planB = self.Y_test

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
                String, '/goal_type', 
                self.need_next_object_callback, 10)
            self._pose_pub = self.create_subscription(
                Pose2D, '/odom_pose', self.odometry_yaw_callback, 10)
            self.create_subscription(
                PoseStamped, '/motion/goal', self.goal_callback, 10)
            
            self.ota_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = qos_profile) # ota = object_tuck_arm
            self.picklift_pub = self.node.create_publisher(Bool, '/picklift/succeded', 10)
            self.trigger_mapping = self.node.create_publisher(Bool, '/do_mapping', 10)
            

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
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.box_x, self.box_y = None, None


    def initialise(self):
        self.arm_moving = False
        self.arm_tucked = False
        self.trigger_mapping.publish(Bool(data = False))
        self.timer = self.node.create_timer(3, self.timer_callback)
        
        
    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked 
            If the next goal is to place an object, the arm will be tucked. Otherwise, we want to tick the picking behavior"""
        if self.next_goal == 'Object':
            #self.get_logger().info(f"Not the time for placing")
            return py_trees.common.Status.FAILURE
        else:
            #self.get_logger().info(f"Placing: started = {self.arm_started}, moving = {self.arm_moving}, tucked = {self.arm_tucked}")
            if not self.arm_started and not self.arm_tucked and not self.arm_moving: #initial condition, before the delay
                return py_trees.common.Status.RUNNING
            
            elif self.arm_started and not self.arm_moving and not self.arm_tucked: #after the timer callback
                # Compute necessary orientation of arm base
                desired_yaw = self.desired_arm_base_angle()
                error = 180*(desired_yaw )/pi #TODO: wrap angles, check limits and uncomment below
                normalized_error = max(-120, min(120, ((error + 180) % 360) - 180))
                self.desired_servo_angles[5] += int(normalized_error*100)
                self.get_logger().info(f'Arm base should rotate {normalized_error} degrees')
                self.arm_moving = True
                self.publish_msg()
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
        #print(f"delay completed")

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
        self.get_logger().info(f"Placing node received '{msg.data}' as next goal")

    def publish_msg(self):
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
            data_offset=0)    
          
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

        # adjust the robot position to favor the grasping of the object
    # def do_adjust_yaw(self, object_x, object_y):
    #     transform = self.tf_buffer.lookup_transform(
    #     'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
    #     )
    #     current_yaw = 2 * atan2(transform.transform.rotation.z, transform.transform.rotation.w)
    #     desired_yaw = atan2(object_y, object_x)
    #     yaw_diff = desired_yaw - current_yaw

    #     pose = PoseStamped()
    #     pose.header.stamp = self.get_clock().now().to_msg()
    #     pose.header.frame_id = 'map'
    #     pose.pose.position.x = transform.transform.translation.x
    #     pose.pose.position.y = transform.transform.translation.y
    #     pose.pose.position.z = transform.transform.translation.z
    #     pose.pose.orientation.z = sin(yaw_diff / 2)
    #     pose.pose.orientation.w = cos(yaw_diff / 2)

    #     self.motion_goal_publisher_.publish(pose)