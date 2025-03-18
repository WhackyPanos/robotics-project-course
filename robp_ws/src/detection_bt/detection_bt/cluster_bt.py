import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ClusterBT(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="clustering_bt"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, name)

    def setup(self, **kwargs):
        """ Initialize ROS publishers and subscribers. """
        self.node = kwargs['node']
        self.cluster_found = None

        # Subscribe to clustering result
        self.result_sub = self.node.create_subscription(
            Bool, "/detection/result", self.result_callback, 10)

        # Publish to request clustering
        self.request_pub = self.node.create_publisher(Bool, "/detection/request", 10)

    def initialise(self):
        """ Reset cluster_found before triggering clustering. """
        self.cluster_found = None
        msg = Bool()
        msg.data = True
        self.request_pub.publish(msg)

    def update(self):
        """ Check clustering result and return status. """
        if self.cluster_found is None:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.cluster_found else py_trees.common.Status.FAILURE

    def terminate(self):
        return py_trees.common.Status.SUCCESS

    def result_callback(self, msg):
        """ Receive clustering result and store it. """
        self.cluster_found = msg.data