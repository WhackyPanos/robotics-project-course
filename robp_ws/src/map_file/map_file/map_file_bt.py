import py_trees
import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from .map_file import Map_file
=======
from std_msgs.msg import Bool
>>>>>>> origin/collection_bt

class MapFileBT(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="map_file_bt"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, name)
<<<<<<< HEAD
        self.map_file_node = Map_file()

    def setup(self, **kwargs):
        pass

    def initialise(self):
        pass

    def update(self):
        return py_trees.common.Status.SUCCESS if self.map_file_node.perform_map_file_update() else py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        pass
=======

    def setup(self, **kwargs):
        """ Initialize ROS publishers and subscribers. """
        self.node = kwargs['node']
        self.map_updated = None

        # Subscribe to clustering result
        self.result_sub = self.node.create_subscription(
            Bool, "/map_file/result", self.result_callback, 10)

        # Publish to request clustering
        self.request_pub = self.node.create_publisher(Bool, "/map_file/request", 10)

    def initialise(self):
        self.map_updated = None
        msg = Bool()
        msg.data = True
        self.request_pub.publish(msg)

    def update(self):
        if self.map_updated is None:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.map_updated else py_trees.common.Status.FAILURE

    def terminate(self):
        return py_trees.common.Status.SUCCESS

    def result_callback(self, msg):
        self.map_updated = msg.data
>>>>>>> origin/collection_bt
