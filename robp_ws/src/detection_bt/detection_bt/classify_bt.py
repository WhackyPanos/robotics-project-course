import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ClassifyBT(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="classifier_bt"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, name)
<<<<<<< HEAD
        self.class_found = None
=======
>>>>>>> origin/collection_bt

    def setup(self, **kwargs):
        """ Initialize ROS publishers and subscribers. """
        self.node = kwargs['node']
<<<<<<< HEAD
=======
        self.class_found = None
>>>>>>> origin/collection_bt

        # Subscribe to clustering result
        self.result_sub = self.node.create_subscription(
            Bool, "/classification/result", self.result_callback, 10)

        # Publish to request clustering
        self.request_pub = self.node.create_publisher(Bool, "/classification/request", 10)

    def initialise(self):
<<<<<<< HEAD
=======
        
>>>>>>> origin/collection_bt
        self.class_found = None
        msg = Bool()
        msg.data = True
        self.request_pub.publish(msg)

    def update(self):
<<<<<<< HEAD
=======
        
>>>>>>> origin/collection_bt
        if self.class_found is None:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.class_found else py_trees.common.Status.FAILURE

<<<<<<< HEAD
    def terminate(self, new_status: py_trees.common.Status):
        pass

    def result_callback(self, msg):
=======
    def terminate(self):
        return py_trees.common.Status.SUCCESS

    def result_callback(self, msg):
        
>>>>>>> origin/collection_bt
        self.class_found = msg.data