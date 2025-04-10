import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BoxSegmentationBT(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="box_segmentation_bt"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, name)
        self.box_found = None

    def setup(self, **kwargs):
        """ Initialize ROS publishers and subscribers. """
        self.node = kwargs['node']

        # Subscribe to clustering result
        self.result_sub = self.node.create_subscription(
            Bool, "/arm_camera/box_result", self.result_callback, 10)

        # Publish to request clustering
        self.request_pub = self.node.create_publisher(Bool, "/arm_camera/box_request", 10)

    def initialise(self):
        self.box_found = None
        msg = Bool()
        msg.data = True
        self.request_pub.publish(msg)

    def update(self):
        if self.box_found is None:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.box_found else py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        pass

    def result_callback(self, msg):
        self.box_found = msg.data

