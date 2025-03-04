#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from handle_objects.pick_objects import InitTuckArm, ObjTuckArm, Move2Pick, DetectObject
from path_planner.path_planner import CarrotPlanner
from behavior_tree.goCollect_bhv import goCollect
from rclpy.executors import MultiThreadedExecutor

class PickBT(Node):
    def __init__(self) -> None:
        super().__init__('behavior_tree')

        # Create the root behavior
        root = self.create_root()

        # Create and initialize the behavior tree
        self.tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)

    def create_root(self):    
        # Create the root as a Sequence node (default memory=False is fine here)
        root = py_trees.composites.Sequence(name="Root", memory= False)

        # Initialize behaviors and pass ROS node to them
        collect = goCollect()
        move_to_pick = Move2Pick()
        obj_tuck_bhv = ObjTuckArm()
        
        root.add_children([collect, move_to_pick, obj_tuck_bhv]) #, move_to_pick, obj_tuck_bhv
        #root.add_children([obj_tuck_bhv])
        
        return root

def main(args=None):
    rclpy.init(args=args)

    # Initialize the PickBT node
    node = PickBT()

    # Setup the behavior tree with a timeout for setup (10 seconds)
    node.tree.setup(timeout=10.0, node=node)

    # Continuously tick the behavior tree
    node.tree.tick_tock(period_ms=1000)

    # Spin the node to keep it alive
    try:
        rclpy.spin(node)  # This keeps the node alive
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully


    # Shutdown the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()

