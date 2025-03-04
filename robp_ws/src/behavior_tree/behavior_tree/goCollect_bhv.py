#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import py_trees
from path_planner.collectObjectMS2 import CollectObjectMS2 
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class goCollect(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="goCollect"):
        super().__init__(name=name)
        # define goal position for now
        self.goal = PointStamped()
        self.goal.header.frame_id = "map"
        self.goal.point.x = 0.5
        self.goal.point.y = 0.0
        self.goal.point.z = 0.0
        self.planner = CollectObjectMS2()
        

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
           a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs["node"]
        rclpy.get_global_executor().add_node(self.planner)  # Add to the ROS executor
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.goal_publisher = self.node.create_publisher(PointStamped, '/temp_goal', qos_profile)
        self.node.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        self.point_reached = False
        self.moving = False
        
    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter."""
        # in the first tick, we'll publish the goal with a time delay (4 seconds rn)
        self.timer = self.node.create_timer(10.0, self.publish_goal)   

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        if self.point_reached:
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
        return py_trees.common.Status.SUCCESS

    def publish_goal(self):
        """ Publish the goal position to the robot x seconds after the behavior initialized"""
        self.node.get_logger().info("Goal Published")
        self.goal_publisher.publish(self.goal)
        self.moving = True
        self.timer.cancel()

    def goal_reached_callback(self, msg):
        if self.moving and msg.data: # robot is moving and has reached the goal point
            self.point_reached = True