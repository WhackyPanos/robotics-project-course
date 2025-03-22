import py_trees
import rclpy
from rclpy.node import Node
from localization.localization_transform import Localization
from std_msgs.msg import Bool



class Localization_bhv(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="Localization"):
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        self.localization_node = Localization()

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
        a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs["node"]
        self.icp_time = 0.2 #every 1 s
        self.node.create_subscription(
            Bool, "/icp/activate", self.localization_activate_callback, 10)
        self.localization_activate = True
        self.localization_timer = None


    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
        status is not RUNNING thereafter."""
        pass  


    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        if self.localization_activate:
            self.localization_timer = self.node.create_timer(self.icp_time, self.localization_node.icp_master)
            return py_trees.common.Status.RUNNING
        else:
            try:
                self.localization_timer.cancel()
            except:
                pass
            return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        pass

    def localization_activate_callback(self, msg):
        self.localization_activate = msg.data
        return
  