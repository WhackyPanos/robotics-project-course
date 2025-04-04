import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ArmSegmentationBT(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="arm_segmentation_bt"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, name)
        self.object_found = None
        self.checkout_time = 8.0

    def setup(self, **kwargs):
        """ Initialize ROS publishers and subscribers. """
        self.node = kwargs['node']

        # Subscribe to clustering result
        self.result_sub = self.node.create_subscription(
            Bool, "/arm_cam/result", self.result_callback, 10)

        # Publish to request clustering
        self.request_pub = self.node.create_publisher(Bool, "/arm_cam/request", 10)

    def initialise(self):
        self.object_found = None
        msg = Bool()
        msg.data = True
        self.request_pub.publish(msg)
        self.init_time = self.get_clock().now().nanoseconds / 1e9

    def update(self):
        curr_time = self.get_clock().now().nanoseconds / 1e9
        if self.object_found is None:
            if curr_time - self.init_time < self.checkout_time:
                return py_trees.common.Status.RUNNING
            else:
                self.node.get_logger().warn(f"Arm camera did not work, using transforms")
                return py_trees.common.Status.SUCCESS 
        return py_trees.common.Status.SUCCESS if self.object_found else py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        pass

    def result_callback(self, msg):
        self.object_found = msg.data

