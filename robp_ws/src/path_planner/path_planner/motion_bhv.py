import py_trees
import rclpy
from rclpy.node import Node
from motion import MotionNode

class NavigateToGoal(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="NavigateToGoal"):
        super().__init__(name=name)
        self.motion_node = MotionNode() # create a motion node object

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
        a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']
        rclpy.get_global_executor().add_node(self.motion_node)

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
        status is not RUNNING thereafter."""  
        self.get_logger().info("Motion behavior initialized")

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        if self.motion_node.is_path:
            if self.motion_node.path_reached:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        else:
            if self.motion_node.goal_reached_flag:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING

    # def timer_callback(self):
    #     """ Callback function for the watchdog timer """
    #     return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.get_logger().info(f"Terminating PublishOccupancyGridBT")