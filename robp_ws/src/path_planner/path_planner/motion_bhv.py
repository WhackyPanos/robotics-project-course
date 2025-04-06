import py_trees
import rclpy
from rclpy.node import Node
from .motion import MotionNode


class NavigateToGoal(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="NavigateToGoal"):
        # Initialize Behaviour (PyTrees) and Node (ROS2)
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, name)  # Explicitly initialize ROS2 Node
        
        self.motion_node = MotionNode() # create a motion node object
        self.timer = None
        self.wait_for_goal_flag = False

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
        a parallel checking for a valid policy configuration after children have been added or removed"""
        #rclpy.get_global_executor().add_node(self.motion_node)

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
        status is not RUNNING thereafter."""  
        self.get_logger().info("Motion behavior initialized")
        self.motion_node.path_reached, self.motion_node.goal_reached_flag = False, False # FRANCISCO CHANGED
        self.wait_for_goal_flag = False
        self.motion_node.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.motion_node.prev_angle_diff = 0.0
        self.timer = self.create_timer(3.0, self.wait_for_goal)
        self.get_logger().info(f"Motion waiting for timer ...")

    def update(self):
        """ Behavior Tree execution step. Called whenever the node is ticked """
        #self.get_logger().info(f"Is goal: {self.motion_node.is_goal}, Goal reached: {self.motion_node.goal_reached_flag}")
        #self.get_logger().info(f"Is path: {self.motion_node.is_path}, Path reached: {self.motion_node.path_reached}")
        #self.get_logger().info(f"Motion ticked. Is goal: {self.motion_node.is_goal}, Goal reached: {self.motion_node.goal_reached_flag}")
        if not self.wait_for_goal_flag:
            return py_trees.common.Status.RUNNING 
        else:
            if self.motion_node.is_path:
                if self.motion_node.path_reached:
                    self.motion_node.is_path = False
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.RUNNING if self.motion_node.navigate_to_goal() else py_trees.common.Status.FAILURE
            elif self.motion_node.is_goal:
                if self.motion_node.goal_reached_flag:
                    #self.motion_node.is_goal = False
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.RUNNING if self.motion_node.navigate_to_goal() else py_trees.common.Status.FAILURE
            else: 
                #self.get_logger().info(f"Is goal: {self.motion_node.is_goal}, Goal reached: {self.motion_node.goal_reached_flag}")

                return py_trees.common.Status.FAILURE

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
        # self.get_logger().info(f"Terminating Motion behavior")
        pass

    def wait_for_goal(self):
        self.wait_for_goal_flag = True