import py_trees
import rclpy
from rclpy.node import Node
from .random_workspace_point import RandomPoint


class PointGenerator(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Goal"):
        # Initialize Behaviour (PyTrees) and Node (ROS2)
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        
        self.random_point_node = RandomPoint() # create a motion node object

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
        a parallel checking for a valid policy configuration after children have been added or removed"""
        #rclpy.get_global_executor().add_node(self.random_point_node)

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
        status is not RUNNING thereafter."""  
        self.get_logger().info("Random point behavior initialized")

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        return py_trees.common.Status.SUCCESS if self.random_point_node.generate_new_point() else py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.get_logger().info(f"Terminating random point behavior")