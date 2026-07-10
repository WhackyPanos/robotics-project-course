import py_trees
import rclpy
from rclpy.node import Node
from .map_file import Map_file

class MapFileBT(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name="map_file_bt"):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, name)
        self.map_file_node = Map_file()

    def setup(self, **kwargs):
        pass

    def initialise(self):
        pass

    def update(self):
        return py_trees.common.Status.SUCCESS if self.map_file_node.perform_map_file_update() else py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        pass
