#!/usr/bin/env python

import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
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
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from math import pi, acos, atan2, atan, cos, sin, sqrt
import numpy as np 

class Move2Pick(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Move2Pick"):
        super().__init__(name=name)

        self.ik_solver = IKNode()
        self.to_frame_rel = 'arm_base_link'
        self.from_frame_rel = 'map'
        self.X, self.Y,  self.x, self.y, self.z, self.target = None, None, None, None, None, None
        self.thresholds = [10**-4,10**-3,10**-2]
        # self.initial_guesses = [
        #     [0, 80*pi/180, 80*pi/180, 0, 0, 0],  
        #     [10*pi/180, 75*pi/180, 40*pi/180, 10*pi/180, 0, 0],  
        #     [0, 70*pi/180, 50*pi/180, 20*pi/180, 0, 0], 
        #     [0,0,0,0,0,0], 
        #     [-5*pi/180, 85*pi/180, 60*pi/180, -10*pi/180, 0, 0],  
        #     [0, 90*pi/180, 70*pi/180, 10*pi/180, 0, 0],  
        #     [5*pi/180, 60*pi/180, 80*pi/180, -5*pi/180, 0, 0],  
        #     [-10*pi/180, 100*pi/180, 90*pi/180, 5*pi/180, 0, 0],  
        #     [0, 95*pi/180, 85*pi/180, -5*pi/180, 0, 0],  
        #     [0, 110*pi/180, 75*pi/180, 0, 0, 0],  
        #     [-5*pi/180, 120*pi/180, 60*pi/180, -10*pi/180, 0, 0]  
        # ]
        self.initial_guesses = [[0,0,0,0,0,0]]
        self.angle_threshold = 80

        # current angles and joints limits in the arm domain
        self.current_angles, self.desired_servo_angles = None, None

        # joint limits in the arm domain
        self.lb_angles = [0.0,0.0,30.0,30.0,60.0,0.0]
        self.ub_angles = [90.0,240.0,210.0,210.0,180.0,240.0]

        # joint limits in normal domain
        self.lb_q = [-120.0,-60.0,-90.0,-90.0,-120.0,0.0]
        self.ub_q = [120.0,60.0,90.0,90.0,120.0,0.0]

        self.obj_tuck_arm_time = 3000 # in ms   


    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']
        self.dopmc_available = False # dopac = detected object position in the arm camera (frame)
        self.ready2move= False
        self.arm_moving = False
        self.arm_tucked = False  
        self.object_grasped = False  

        # Initialize the transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # get object position in the main camera frame
        """
        self.dopmc_subscriber = self.node.create_subscription(
            ObjectPosition,
            '/detected_object_pose/main_camera',
            self.dopmc_callback,
            10)
        """
        self.node.create_subscription(PointStamped, '/temp_goal', self.dopmc_callback, 10)
        #self.pub = self.node.create_publisher(ObjectPosition, '/detected_object_pose/main_camera', 10)

        # initialize servo publisher and servo sensor subscriber to send commands to arm
        self.ota_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = 10) # ota = object_tuck_arm
            
        self.servo_angles_subscriber_ = self.node.create_subscription(
                JointState,
                '/servo_pos_publisher',
                self.servo_angles_callback,
                10
            )     
        
    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        """ Wait x seconds to initiate arm movement (and publish object position later on)"""
        self.timer = self.node.create_timer(1, self.init_arm_movement)

        """ include this later on
        msg = ObjectPosition()
        msg.x = 0.14
        msg.y = -0.05
        msg.object_type = 'S'
        self.pub.publish(msg)
        """
         
    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """
            #solve_ik(self, target_pose, provided_initial_guess, eps=5e-3, maxiter=100000)
            #self.node.get_logger().info(f'Object ({self.X, self.Y}) in map -> ({self.x,self.y,self.z}) in arm_base')

            # if object position is available, compute target and perform inverse kinematics
            
            if self.ready2move and not self.arm_moving and not self.arm_tucked : # if necessary, self.x is not None and self.y is not None and self.z is not None and 
                self.object_transform() # tranform object position from map frame to arm base frame
                self.node.get_logger().info(f"Trying to reach {[self.x, self.y, self.z]}")
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
                self.node.get_logger().warn(f"COULD NOT GET SOLUTION, trying plan B")

                # if everything fails, try a simpler approach
                msg = self.pick_planB(self.x, self.y, self.z)
                if msg is not None:
                    self.ota_publisher_.publish(msg)
                    #self.move_timer = self.node.create_timer(self.obj_tuck_arm_time/1000 + 3.0, self.wait_for_movement)
                    self.arm_moving = True
                    return py_trees.common.Status.RUNNING
                else:
                    return py_trees.common.Status.FAILURE
            
            # if arm is moving but not in grasp position, return keep running
            elif self.arm_moving and not self.arm_tucked:
                return py_trees.common.Status.RUNNING 
            
            # if arm is in grasp position,  start grasping 
            elif self.arm_tucked and not self.object_grasped and not self.arm_moving:
                self.node.get_logger().info(f"Arm Moving = {self.arm_moving}, Arm Tucked = {self.arm_tucked} and grasped = {self.object_grasped}")
                msg = Int16MultiArray()
                msg.layout = MultiArrayLayout(
                    dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
                    data_offset=0
                )  
                times = [self.obj_tuck_arm_time] * 6
                self.desired_servo_angles[0] = 10000 #close gripper
                msg.data = self.desired_servo_angles + times
                self.ota_publisher_.publish(msg)
                #self.move_timer = self.node.create_timer(self.obj_tuck_arm_time/1000 + 3.0, self.wait_for_movement)
                self.arm_moving = True
                #self.node.get_logger().info(f"Publishing the anglee {self.desired_servo_angles}")
                return py_trees.common.Status.RUNNING
            
            elif self.arm_tucked and not self.object_grasped and self.arm_moving:
                return py_trees.common.Status.RUNNING 
            
            # if object is grasped, return success
            elif self.object_grasped:
                self.node.get_logger().info(f"Sucess, lifting arm now")
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

    # def wait_for_movement(self):
    #     if self.arm_moving and self.arm_tucked:
    #         self.arm_moving = False
    #         self.object_grasped = True
    #     elif self.arm_moving and not self.arm_tucked:
    #         self.arm_moving = False
    #         self.arm_tucked = True
    #         self.node.get_logger().warn("Concluding this arm movement due to time limit, angles are not accurate!")
    #     self.move_timer.cancel()


    def dopmc_callback(self,msg):
        # store object position in map frame
        self.X = msg.point.x # later on change to msg.x
        self.Y = msg.point.y # later on change to msg.y
        obj = "C" # change later on to msg.object_type

    def object_transform(self):
        # get transform from map frame to frame of the arm base
        time = rclpy.time.Time() #retrieve most recent transform ig
        try:
            t = self.tf_buffer.lookup_transform(
                self.to_frame_rel,
                self.from_frame_rel,
                time)
        except TransformException as ex:
            self.node.get_logger().info(
                f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}')
            return
        
        # do transformation   
        object_MF = PointStamped()
        object_MF.header.frame_id = 'map'
        object_MF.point.x = self.X
        object_MF.point.y = self.Y
        object_MF.point.z = 0.0         
        object_ABF = tf2_geometry_msgs.do_transform_point(object_MF, t)
        self.x = object_ABF.point.x
        self.y = object_ABF.point.y
        self.z = object_ABF.point.z

        #self.x = 0.16 # DELETE AS SOON AS WE HAVE A VALID MAP FRAME
        #self.y = 0
        
        # define target with kdl instance
        self.node.get_logger().info(f'Object ({self.X, self.Y}) in map -> ({self.x,self.y,self.z}) in arm_base')

    def servo_angles_callback(self, msg):
        """ Callback to check if arm is in a good enough position while moving. improve later on"""
        self.current_angles = msg.position
        if self.arm_moving and not self.arm_tucked and not self.object_grasped:
            self.arm_tucked = True
            self.arm_moving = False
            for i in range(1, len(self.current_angles)) :
                if abs(self.desired_servo_angles[i] -self.current_angles[i]) > self.angle_threshold:
                    self.node.get_logger().info(f"Arm still moving, error of {abs(self.desired_servo_angles[i] -self.current_angles[i])} in joint {i+1}")
                    self.arm_tucked = False
                    self.arm_moving = True
                    break

        elif self.arm_moving and self.arm_tucked and not self.object_grasped:
            self.desired_servo_angles[0] = 10000
            self.object_grasped = True
            self.arm_moving = False
            for i in range(1, len(self.current_angles)) :
                if abs(self.desired_servo_angles[i] -self.current_angles[i]) > self.angle_threshold:
                    self.node.get_logger().info(f"Ongoing grasping {abs(self.desired_servo_angles[i] -self.current_angles[i])}")
                    self.object_grasped = False
                    self.arm_moving = True
                    break
        
    def publish_angles(self, ik_angles):
        """ Publish corrected angles to the arm after having computing inverse kinematics"""
        # retrieve current angles (supposedly arm is stretched) and compute corrected angles to publish
        init_angles = np.multiply(0.01,np.array(self.current_angles)) # from servo sensors, pass from cdeg to deg
        desired_angles = self.wrap_angle((180/pi)*np.array(ik_angles[::-1]), "degrees") # convert ik_angles to angles bw -180 and 180        
        res_angles = init_angles + np.multiply(ik_angles[::-1], [1,1,-1,1,-1,1]) # CHECK THIS VECTOR AGAIN
        pub_angles, out_limits = self.check_limits(res_angles, self.lb_angles, self.ub_angles)

        self.node.get_logger().info(f"Current angles are {init_angles}") 
        self.node.get_logger().info(f"IK angles after wrapping are {desired_angles}")
        self.node.get_logger().info(f"Resulting angles {res_angles}")

        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
            data_offset=0
        )      
        times = [self.obj_tuck_arm_time] * 6
        self.desired_servo_angles = [int(angle*100) for angle in pub_angles]
        msg.data = self.desired_servo_angles + times
        msg.data[0] = 2600 #keep gripper open
        

        if not out_limits:
            self.node.get_logger().info(f" Angles will be published with ik_solver = {self.desired_servo_angles}")
            self.ota_publisher_.publish(msg)
            #self.move_timer = self.node.create_timer(self.obj_tuck_arm_time/1000 + 3.0, self.wait_for_movement)
        else:
            self.node.get_logger().info(f" Angles out of limits, trying again ")
            
        return out_limits

    def wrap_angle(self, angles, units: str):
        if units == "radians":
            return [(angle + pi) % (2 * pi) - pi for angle in angles]
        else:
            return [(angle + 180) % 360 - 180 for angle in angles]
        
    def check_limits(self, angles, lb, ub):
        final_angles = angles
        out_limits = False
        for i,x in enumerate(angles):
            if x<lb[i]:
                final_angles[i] = lb[i]+1
                out_limits = True
            elif x>ub[i]:
                final_angles[i] = ub[i]-1
                out_limits = True
        return final_angles, out_limits

    def pick_planB(self, x, y, z):  

        # we will iterate with different orientations for the 1st link. 
        count = 0
        for i in range(90,25,-2):
            # Initiate constants
            good_flag = True
            height = 93*(10**-3)
            l0 = 101*(10**-3)
            l1 = 95*(10**-3)
            l2 = 168*(10**-3)+ 0.0  #1cm to account for object height and avoid colliding with ground
            
            q = [0.0, 0.0, 0.0, 0.0] #joint angles, from base to last y joint, respectively

            # Account for the geometry (different frame)
            new_z = -(abs(z) + height+ l0*sin(i*pi/180))
            new_x = sqrt(x**2 + y**2)-l0*cos(i*pi/180)
            theta = atan2(y,x) #orientation of the arm base

            # Compute & assign angles
            k = (new_x)**2 + new_z**2 - l1**2 - l2**2 # Auxiliary constant
            try:
                q2 = acos(k/(2*l1*l2)) # acos returns between 0 and pi
            except:
               #self.node.get_logger().info(f" x = {new_x} and z = {new_z} not reacheable ({abs(new_z), self.a, self.b, l0*sin(i*pi/180)})")
                continue           
            q[0] = theta*180/pi
            q[1] = 90 - i
            q[2] = (atan2(new_z, new_x) - atan2(l2*sin(q2), l1 + l2*cos(q2)))*180/pi
            q[3] = q2*180/pi

            # check for limits and end loop if everything is ok
            for j in range(len(q)):
                if q[j] < self.lb_q[j] or q[j] > self.ub_q[j]:
                    good_flag = False
                    self.node.get_logger().info(f"Joint {j} angle is outside limits (angle = {q[j+1]})")
            if good_flag == True:
                if count == 0: # skip one more step to avoid joint limits
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
                self.desired_servo_angles[0] = 2600

                # update with plan_b
                self.desired_servo_angles[5] = self.desired_servo_angles[5] + int(100*q[0])   
                self.desired_servo_angles[4] = self.desired_servo_angles[4] + int(-100*q[1])  
                self.desired_servo_angles[3] = self.desired_servo_angles[3] + int(-100*q[2])
                self.desired_servo_angles[2] = self.desired_servo_angles[2] + int(-100*q[3])          
                msg.data = self.desired_servo_angles + times
                return msg
            
        # if no solution was found, msg is invalid
        msg = None
        self.node.get_logger().error("Plan B did not work, returning FAILURE")
        return msg

 # -----------------------------------------------------------------------------------------------------------------------


class ObjTuckArm(py_trees.behaviour.Behaviour): # this class is a py_tree node and a ros node
    def __init__(self, name="ObjTuckArm"):
        super().__init__(name=name)
        #self.logger.debug("ObjTuckArm was called.")
        #self.cached_context = None
    
    def setup(self, **kwargs):
            """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
            print("Setting up ObjTuckArm node.")
            self.node = kwargs['node']
            self.arm_started = False
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,  # Ensures message delivery
                durability=DurabilityPolicy.VOLATILE,    # No history saved for late subscribers
                depth=100  # Stores up to 10 messages before dropping old ones
                )

            self.ota_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = qos_profile) # ota = object_tuck_arm
            
            self.servo_angles_subscriber_ = self.node.create_subscription(
                JointState,
                '/servo_pos_publisher',
                self.servo_angles_callback,
                10
            )  
            self.obj_tuck_arm_time = 2000 # in ms            
             
            # init tuck arm angles
            #self.desired_servo_angles = [45, 230, 80.5232360099144, 201.20685353937313, 68.846193182243155, 139.65382600499612]
            self.desired_servo_angles = [12000]*6
            self.desired_servo_angles[0] = 10000 # gripper is different
            
            # self.solve_fk([-15.706226190621685, -58.89322565161596, 89.19116421034657, -83.70501671216269, -6.281438203134586, -15.707868506470371])

            #self.desired_servo_angles[5] = 6000 # gripper is different
            # obj tuck arm angles
            #self.desired_servo_angles[4] = 6000  # servo 5
            #self.desired_servo_angles[3] = 20000   # servo 4
            #self.desired_servo_angles[2] = 8000 # servo 3
            self.angle_threshold = 100 #1 degree  

    def servo_angles_callback(self, msg):
        current_angles = msg.position
        if self.arm_started and self.arm_moving:
            self.arm_tucked = True
            self.arm_moving = False
            for i in range(1, len(current_angles)) :
                if abs(self.desired_servo_angles[i] -current_angles[i]) > self.angle_threshold:
                    print(f"Arm still moving, error of {abs(self.desired_servo_angles[i] -current_angles[i])}")
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
        self.arm_moving = False
        self.arm_tucked = False
        self.timer = self.node.create_timer(3, self.timer_callback)
        
    def timer_callback(self):
        self.arm_started = True
        self.timer.cancel()
        print(f"delay completed")
         
    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """

        print(f"started = {self.arm_started}, moving = {self.arm_moving}, tucked = {self.arm_tucked}")

        if not self.arm_started and not self.arm_tucked and not self.arm_moving: #initial condition, before the delay
            return py_trees.common.Status.RUNNING
        
        elif self.arm_started and not self.arm_moving and not self.arm_tucked: #after the timer callback
            self.publish_msg()
            print("Publishing INITIAL tuck arm command.")
            self.arm_moving = True
            return py_trees.common.Status.RUNNING  # Keep running while the arm moves
        
        elif self.arm_started and self.arm_moving and not self.arm_tucked:
            #self.publish_msg()
            print("Arm moving.")
            return py_trees.common.Status.RUNNING # Keep running while the arm moves
        
        elif self.arm_tucked:
            print("Arm is in a good position")
            self.arm_started,self.arm_tucked, self.arm_moving= False, False, False
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





 # -----------------------------------------------------------------------------------------------------------------------

class DetectObject(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Detect Object"):
        super().__init__(name=name)
        #py_trees.behaviour.Behaviour.__init__(self, name)
        #Node.__init__(self, "pick_node")  # ROS 2 node initialization
        self.ik_solver = IKNode()

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']
        self.dopac_available = False # dopac = detected object position in the arm camera (frame)
        self.dopmc_available = False # dopmc = detected object position in the main camera (frame)

        # clients that requests the position in the main camera frame (probably from map file)
        self.dopmc_client = self.node.create_client(
            service_type = ObjectPositionSrv,
            service_name = '/detected_object_pose/main_camera')
        self.dopac_client = self.node.create_client(
            service_type = ObjectPositionSrv,
            service_name = '/detected_object_pose/arm_camera')
        self.dopac_publisher = self.node.create_publisher(
            
        )
        
        
    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""    
    
        while not self.dopmc_client.wait_for_service(timeout_sec=2.0) or not self.dopac_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting for detected object position ...')
        
        # initializing requests
        self.arm_request = ObjectPositionSrv.Request()
        self.main_request = ObjectPositionSrv.Request()

        # sending requests
        self.dopmc_client.send_obj_position_request()
        self.dopac_client.send_obj_position_request()
         
    def send_dopmc_request(self): 
        self.main_request.wantObjectPose = True
        self.dopmc_client.call_async(self.main_request).add_done_callback(self.dopmc_response_callback)

    def send_dopac_request(self): 
        self.arm_request.wantObjectPose = True  
        self.dopac_client.call_async(self.arm_request).add_done_callback(self.dopac_response_callback)

    def dopmc_response_callback(self, future):
        """ Callback for the response from the main camera service. """
        try:
            response = future.result()
            self.get_logger().info(f"Main camera pose response: {response.object_pose}")
            # Do your computations or further processing here based on the response from the main camera
            self.dopmc_available = True  # Mark that  now available
        except Exception as e:
            self.get_logger().error(f"Failed to get response from main camera service: {e}")

    def dopac_response_callback(self, future):
        """ Callback for the response from the arm camera service. """
        try:
            response = future.result()
            self.get_logger().info(f"Arm camera pose response: {response.object_pose}")
            # Do your computations or further processing here based on the response from the arm camera
            self.dopac_available = True  # Mark that now available
        except Exception as e:
            self.get_logger().error(f"Failed to get response from arm camera service: {e}")


         
    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """
            #self.ik_solver.solve_ik()
            if not self.dopac_available:
                return py_trees.common.Status.RUNNING
            elif self.dopac_available:
                
                target_pose = 1; #TODO
                #self.ik_solver.solve_ik()

                # TODO
                return py_trees.common.Status.RUNNING
            
            elif self.dopac_available is None:
                return py_trees.common.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

 # -----------------------------------------------------------------------------------------------------------------------


class InitTuckArm(py_trees.behaviour.Behaviour): # this class is a py_tree node and a ros node
    def __init__(self, name="InitTuckArm"):
        super().__init__(name=name)

    def setup(self, **kwargs):
            """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
            print("Setting up ObjTuckArm node.")
            self.node = kwargs['node']
            self.ita_publisher_ = self.node.create_publisher(
                msg_type = Int16MultiArray,
                topic = '/multi_servo_cmd_sub',
                qos_profile = 10) # ota = object_tuck_arm
            self.init_tuck_arm_time = 1000 # in ms
            self.arm_moving = True

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""
        print("Initializing ObjTuckArm behavior.")
        if self.arm_moving:
            self.arm_moving = False
            self.timer = self.node.create_timer(self.init_tuck_arm_time*2, self.finish_tuck_arm)

        
    def finish_tuck_arm(self):
        """ Callback to mark arm movement as complete after delay. """
        print("Arm tucking finished.")
        self.arm_moving = False
        self.timer.cancel()  # Stop the timer
         
         
    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        print(f"Ticking ObjTuckArm, it is {self.arm_moving} that the arm is moving")
        if self.arm_moving:
            return py_trees.common.Status.RUNNING  # Keep running while the arm moves

        else:
            # Publish the tuck command
            msg = Int16MultiArray()
            msg.layout = MultiArrayLayout(
                dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
                data_offset=0
            )      
            angles = [12000] * 6
            angles[3] = 20000
            angles[2] = 6000
            times = [self.init_tuck_arm_time] * 6
            msg.data = angles + times
            print(f"About to publish message")
            self.ita_publisher_.publish(msg)
            print("Publishing tuck arm command.")
            return py_trees.common.Status.SUCCESS
        

"""
    def initialise(self):
        print("Initializing")
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="joint_cmds", size=6, stride=1)],
            data_offset=0
        )      
        times = [self.obj_tuck_arm_time] * 6
        msg.data = self.desired_servo_angles + times
        self.ota_publisher_.publish(msg)
        print("Publishing tuck arm command.")
        self.arm_moving = True
        self.arm_tucked = False
        # Introduce time delay before creating the subscription 
        self.subs_delay_timer = self.node.create_timer(0.2, self.delay_callback)
    def delay_callback(self):
        print("Creating subscriber")
 
        self.node.destroy_timer(self.subs_delay_timer)  # Correct way to remove the timer
"""