#!/usr/bin/env python

import py_trees
import py_trees_ros
import rclpy
import time
#from rclpy.action import ActionClient
#from example_interfaces.action import Move  # Replace with your actual action
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy



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
        if self.arm_started:
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
        self.arm_started = False
        
         
    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """

        if not self.arm_moving and not self.arm_tucked:
            self.publish_msg()
            print("Publishing INITIAL tuck arm command.")
            self.arm_started = True
            self.arm_moving = True
            return py_trees.common.Status.RUNNING  # Keep running while the arm moves
        
        elif self.arm_tucked:
            print("Arm is in a good position")
            self.arm_started,self.arm_tucked, self.arm_moving= False, False, False
            return py_trees.common.Status.SUCCESS

        elif not self.arm_tucked and self.arm_moving and self.arm_started:
            #self.publish_msg()
            print("Arm moving.")
            return py_trees.common.Status.RUNNING
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



 # -----------------------------------------------------------------------------------------------------------------------

class Pick(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Pick"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, "pick_node")  # ROS 2 node initialization


    def setup(self):
         """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""

    def initialise(self):
         """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""
         
    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """

 # -----------------------------------------------------------------------------------------------------------------------
