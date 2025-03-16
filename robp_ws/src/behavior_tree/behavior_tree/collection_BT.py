#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from handle_objects.pick_objects import SetArm, DetectObject, SearchObjectArm, ArmIK
from ppe_bhv import A_MoveRobot, A_PlanPath, I_UpdateObjectList, I_NextObject

from path_planner.path_planner import CarrotPlanner
from behavior_tree.goCollect_bhv import goTo
from rclpy.executors import MultiThreadedExecutor


class CollectionBT(Node):
    def __init__(self) -> None:
        super().__init__('behavior_tree')
        self.filename = '' #introduce name of the text file
        self.objs_list, self.box_list = self.create_lists(self)
        root = self.create_root()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)

    def create_root(self):    
        # Create the root as a Sequence node (default memory=False is fine here)
        root = py_trees.composites.Sequence(name="Root", memory= False)


        # ------------------------ 1st and 2nd steps of sequence
        compute_next_object = I_NextObject()
        plan_path = A_PlanPath()
        move_to_pick = A_MoveRobot()
        update_obj_list = I_UpdateObjectList()

        plan_and_move.add_children([plan_path, move_to_pick])
        path_planning_execution = py_trees.composites.Selector(name="PPE", memory= False)

        plan_and_move = py_trees.composites.Sequence(name="Plan_and_Move", memory= False)
        path_planning_execution.add_children([plan_and_move, update_obj_list])

        # before 3rd step, define every hard-coded set of angles
        arm_camera_tuck_angles = [2600,12000,12000,12000,12000,12000] 
        lift_angles = [2600,12000,12000,12000,12000,12000] 

        # ------------------------ 3rd step
        pick_lift = py_trees.composites.Sequence(name="Pick&Lift", memory= False)

        planA = py_trees.composites.Sequence(name="PlanA", memory= False)
        planB = py_trees.composites.Parallel(name="PlanB", memory= False)

        #plan A
        camera_tuck = SetArm('camera_tuck', arm_camera_tuck_angles) 
        arm_camera_detection = DetectObject()
        move_arm_around = SearchObjectArm()
        search_loop = py_trees.composites.Parallel(name="PlanB", 
                    policy = py_trees.common.ParallelPolicy.SuccessOnOne, children = [arm_camera_detection, move_arm_around])
        arm_ik = ArmIK()
        planA.add_children([camera_tuck, search_loop, arm_ik])
        #plan B TODO

        pick_or_search = py_trees.composites.Selector(name="Pick&Lift", memory= False, children=[planA]) #put planB
        # lift
        lift = SetArm('lift', lift_angles) 

        repeat_picklift = py_trees.decorators.Retry(
            name = 'Repeat_Pick&Lift', 
            child = py_trees.composites.Sequence([pick_or_search, lift]), 
            num_failures = 5)

        # ------------------------ 4th step


        root.add_children([compute_next_object, path_planning_execution]) #, move_to_pick, obj_tuck_bhv
        
        return root
    
    def create_lists(self):
        """ Function to return a list with the objects to collect, as well as the boxes"""
        data = []
        with open(self.filename, 'r') as f:
            for line in f:
                line = line.strip() # remove leading and trailing whitespaces
                words = line.split(',')
                words.pop(0)
                data.append(words)
        header = data[0]
        data.pop(0)

        objs_list = [sublist for sublist in data if sublist[0] != 'B']
        box_list = [sublist for sublist in data if sublist[0] == 'B']

        return objs_list, box_list
    




def main(args=None):
    rclpy.init(args=args)

    # Initialize the PickBT node
    node = CollectionBT()

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