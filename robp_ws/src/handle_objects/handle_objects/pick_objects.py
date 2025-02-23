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

class Move2Pick(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Move2Pick"):
        super().__init__(name=name)

        self.ik_solver = IKNode()
        # Initialize the transform broadcaster, buffer and listener
        self.to_frame_rel = 'arm_base_link'
        self.from_frame_rel = 'map'


    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']
        self.dopmc_available = False # dopac = detected object position in the arm camera (frame)
        self.ready2move= False
        self.X, self.Y,  self.x, self.y, self.z, self.target = None, None, None, None, None, None
        self.thresholds = [10**-4,10**-3,10**-2]

        # -- transformation stuff initialization
        # Initialize the transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # temporary snippet to publish object position in map frame
        self.dopmc_subscriber = self.node.create_subscription(
            ObjectPosition,
            '/detected_object_pose/main_camera',
            self.dopmc_callback,
            10)
        
        self.pub = self.node.create_publisher(ObjectPosition, '/detected_object_pose/main_camera', 10)


        
    def dopmc_callback(self,msg):
        if self.ready2move:
            self.X = msg.x
            self.Y = msg.y
            obj = msg.object_type

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
            self.x = t.transform.translation.x
            self.y = t.transform.translation.x
            self.z = t.transform.translation.x

            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            
            # define target with kdl instance
            self.node.get_logger().info(f'Object ({self.X, self.Y}) in map -> ({self.x,self.y,self.z}) in arm_base')
            #self.target = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(9.963456717271555*0.01, 0, 25.932833880796892*0.01))

        
    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""  
        self.ready2move= True  
        msg = ObjectPosition()
        msg.x = 0.2
        msg.y = 0.0
        msg.object_type = 'S'
        self.pub.publish(msg)

         
    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """
            #solve_ik(self, target_pose, provided_initial_guess, eps=5e-3, maxiter=100000)
            #if self.x is not None and self.y is not None:
                #for i, x in enumerate(self.thresholds):
                    #self.ik_solver.solve_ik()
            self.node.get_logger().info(f'Object ({self.X, self.Y}) in map -> ({self.x,self.y,self.z}) in arm_base')
            return py_trees.common.Status.RUNNING


    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """

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
            self.desired_servo_angles = [12000] * 6
            self.desired_servo_angles[0] = 2600 # gripper is different
            # obj tuck arm angles
            self.desired_servo_angles[4] = 6000  # servo 5
            self.desired_servo_angles[3] = 20000   # servo 4
            self.desired_servo_angles[2] = 8000 # servo 3
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