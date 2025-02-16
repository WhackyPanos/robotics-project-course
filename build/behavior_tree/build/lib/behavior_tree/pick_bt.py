#!/usr/bin/env python

import rclpy
import py_trees
from rclpy.node import Node
from handle_objects.pick.pick_objects import InitTuckArm, ObjTuckArm, Pick


class PickBT(Node):
    def __init__(self):
        super().__init__('behavior_tree')
        self.tree = py_trees.trees.BehaviourTree(self.create_tree())

    def create_tree(self):
        root = py_trees.composites.Sequence("Root")
        pick_action = InitTuckArm()
        root.add_child(pick_action)
        return root

def main(args=None):
    rclpy.init(args=args)
    node = PickBT()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()