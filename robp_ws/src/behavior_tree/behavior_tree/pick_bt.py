#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from handle_objects.pick_objects import InitTuckArm, ObjTuckArm, Move2Pick, DetectObject

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

        #init_tuck_action = InitTuckArm()
        obj_tuck_bhv = ObjTuckArm()
        #pick_obj = Pick()
        #init_tuck_bhv = InitTuckArm()
        move_to_pick = Move2Pick()
        
        # Add behaviors to the root
        #root.add_child(init_tuck_action)
        #root.add_children([move_to_pick])
        root.add_child(move_to_pick)
        
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
    rclpy.spin(node)

    # Shutdown the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
