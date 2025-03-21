import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ClassifyBT(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="classifier_bt"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, name)

    def setup(self, **kwargs):
        """ Initialize ROS publishers and subscribers. """

        # Subscribe to clustering result
        self.result_sub = self.node.create_subscription(
            Bool, "/classification/result", self.result_callback, 10)

        # Publish to request clustering
        self.request_pub = self.node.create_publisher(Bool, "/classification/request", 10)

    def initialise(self):
        self.class_found = None
        msg = Bool()
        msg.data = True
        self.request_pub.publish(msg)

    def update(self):
        
        if self.class_found is None:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.class_found else py_trees.common.Status.FAILURE

    def terminate(self):
        pass

    def result_callback(self, msg):
        self.class_found = msg.data