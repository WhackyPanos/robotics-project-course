import py_trees
import rclpy
from rclpy.node import Node
from .motion import MotionNode
from geometry_msgs.msg import Pose2D, PointStamped
from std_msgs.msg import String
import math

class Rotate(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Rotate"):
        # Initialize Behaviour (PyTrees) and Node (ROS2)
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        
        self.motion_node = MotionNode("Rotate_node") # create a motion node object

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
        a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs["node"]
        self.node.create_subscription(Pose2D, '/odom_pose',  self.get_robot_yaw, 10)
        self.node.create_subscription(String,'/goal_type', self.reset_ticks, 10)
        self.robot_yaw = 0.0
        self.ticks = 0
        self.desired_angle = self.robot_yaw


    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
        status is not RUNNING thereafter."""  
        if self.ticks % 2 == 0:
            self.desired_angle = self.robot_yaw + (self.ticks+1)*math.pi/6 #changed this slightly (francisco)
        else: 
            self.desired_angle = self.robot_yaw - self.ticks* 2*math.pi/6 #changed this slightly (francisco)
        self.ticks +=1
        

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        return py_trees.common.Status.FAILURE if self.motion_node.adjust_yaw(self.desired_angle) else py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        # self.get_logger().info(f"Terminating Motion behavior")
        pass
    def get_robot_yaw(self, msg:Pose2D):
        self.robot_yaw = msg.theta

    def reset_ticks(self, msg:String):
        self.ticks = 0