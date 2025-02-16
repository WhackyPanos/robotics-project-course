#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros_viewer
from rclpy.node import Node
#from handle_objects.handle_objects.pick_objects import InitTuckArm
from handle_objects.pick_objects import InitTuckArm, ObjTuckArm


class PickBT(Node):
    def __init__(self):
        super().__init__('behavior_tree')
        self.tree = py_trees.trees.BehaviourTree(self.create_tree())

    def create_tree(self):
        root = py_trees.composites.Sequence("Root", memory= False)
        pick_action = ObjTuckArm()
        root.add_child(pick_action)
        return root

def main(args=None):
    rclpy.init(args=args)
    node = PickBT()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()