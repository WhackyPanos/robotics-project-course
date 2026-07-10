import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose2D, PointStamped
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ClusterBT(py_trees.behaviour.Behaviour, Node):
    def __init__(self, new_request: Bool, name="clustering_bt"):
        self.name = name + "_new" if new_request else name
        py_trees.behaviour.Behaviour.__init__(self, self.name)
        Node.__init__(self, self.name)
        self.new_req = new_request
        self.new_goal = True
        self.cluster_found = None
        self.robot_x = None
        self.robot_y = None
        self.goal_x = None
        self.goal_y = None

    def setup(self, **kwargs):
        """ Initialize ROS publishers and subscribers. """
        self.node = kwargs['node']

        # Subscribe to clustering result
        self.node.create_subscription(
            Bool, "/detection/result", self.result_callback, 
            rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        
        # Subscribe to odom pose
        self.node.create_subscription(
            Pose2D, "/odom_pose", self.odom_callback, 10)
        
        # Subscribe to goal point
        self.node.create_subscription(
            PointStamped, "/goal_point", self.goal_point_callback, 
            rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        
        # Subscribe to goal type
        self.node.create_subscription(
            String, "/goal_type", self.goal_type_callback, 
            rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        
        # Subscribe to retry
        self.node.create_subscription(
            Bool, "/detection/retry", self.retry_callback, 
            rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))

        # Publish to request clustering
        self.request_pub = self.node.create_publisher(Bool, "/detection/request", 
                                                      rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def initialise(self):
        """ Reset cluster_found before triggering clustering. """
        self.cluster_found = None
        if not self.new_goal:
            return

        if None in [self.robot_x, self.robot_y, self.goal_x, self.goal_y]:
            return

        distance_to_goal = math.dist([self.robot_x, self.robot_y], [self.goal_x, self.goal_y])
        if distance_to_goal < 1.5:
            # self.get_logger().info("Distance to goal < 1.5")
            msg = Bool()
            msg.data = self.new_req
            self.request_pub.publish(msg)
            self.get_logger().info(f"Send detection request with bool {self.new_req}")
        

    def update(self):
        """ Check clustering result and return status. """
        if self.cluster_found is None:
            return py_trees.common.Status.RUNNING
        
        if self.cluster_found:
            if self.new_req:
                self.new_goal = False
            return py_trees.common.Status.FAILURE
            
        if self.new_req:
            self.initialise()
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        pass

    def result_callback(self, msg:Bool):
        """ Receive clustering result and store it. """
        self.cluster_found = msg.data

    def odom_callback(self, msg:Pose2D):
        self.robot_x, self.robot_y = msg.x, msg.y

    def goal_point_callback(self, msg:PointStamped):
        self.goal_x, self.goal_y = msg.point.x, msg.point.y
    
    def goal_type_callback(self, msg:String):
        self.new_goal = True

    def retry_callback(self, msg:Bool):
        self.new_goal = msg.data