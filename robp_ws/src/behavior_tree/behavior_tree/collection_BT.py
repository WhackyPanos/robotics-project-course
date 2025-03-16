#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from handle_objects.pick_objects import Place, Lift, Move2Pick, DetectObject
from path_planner.path_planner import CarrotPlanner
from behavior_tree.goCollect_bhv import goTo
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

        # define pick and place locations
        x_object = 0.5
        y_object = 0.0
        x_box = 0.8
        y_box = 0.3
        # Initialize behaviors and pass ROS node to them
        go_collect = goTo(x_object, y_object, name = 'go_collect')
        move_to_pick = Move2Pick()
        obj_tuck_bhv = Lift()

        go_place = goTo(x_box, y_box, name = 'go_place')
        place = Place()
        
        root.add_children([go_collect, move_to_pick, obj_tuck_bhv, go_place, place]) #, move_to_pick, obj_tuck_bhv
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