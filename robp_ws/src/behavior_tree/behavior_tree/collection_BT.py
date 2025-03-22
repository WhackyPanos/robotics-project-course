#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from handle_objects.pick_objects import SetArm, DetectObject, SearchObjectArm, ArmIK, Place
from behavior_tree.goCollect_bhv import goTo
from rclpy.executors import MultiThreadedExecutor
import os
from .collection_bhv  import UpdateObjectList, ArmTaskSucceeded
from path_planner.motion_bhv import NavigateToGoal
from localization.localization_bhv import Localization_bhv
from mapping.PublishOccupancyGrid_bhv import PublishOccupancyGrid
import time



class CollectionBT(Node):
    def __init__(self) -> None:
        super().__init__('behavior_tree')
        relative_path_to_file = os.path.join("..", "robp_ws/src/behavior_tree", "map_1.tsv")
        self.filename = os.path.realpath(relative_path_to_file) #introduce name of the text file
        self.objs_list, self.box_list = self.create_lists()

        # Root creation
        self.root = py_trees.composites.Sequence(name="Root", memory= False)
        
        self.next_object_bhv = UpdateObjectList(self.objs_list, self.box_list, "next_object")
        self.pub_occupancy_grid = PublishOccupancyGrid()
        self.localization = Localization_bhv()
        self.navigate_to_goal = NavigateToGoal()
        self.path_planner = None #TODO

        self.tuck_arm = SetArm('tuck_arm', [2600,12000,2000,18000,12000,12000])
        #detect_object = DetectObject()
        self.pick_object = ArmIK()
        self.lift = SetArm('lift', [10000,12000,12000,12000,12000,12000])

        self.arm_task_succeeded = ArmTaskSucceeded()

        self.tree = py_trees_ros.trees.BehaviourTree(root=self.root, unicode_tree_debug=False)

    def create_root(self, executor): 
        # Add nodes to executor
        executor.add_node(self.next_object_bhv)
        executor.add_node(self.navigate_to_goal)
        executor.add_node(self.pub_occupancy_grid)
        executor.add_node(self.localization)
        executor.add_node(self.tuck_arm)
        executor.add_node(self.pick_object)
        executor.add_node(self.lift)
        executor.add_node(self.arm_task_succeeded)

        executor.add_node(self.pub_occupancy_grid.occupancy_grid)
        executor.add_node(self.navigate_to_goal.motion_node)
        executor.add_node(self.localization.localization_node)
  
        """ merge behaviors with composites """
        # Path planning and execution for picking
        plan_and_move = py_trees.composites.Sequence(
            name = 'plan_and_move',
            children = [self.navigate_to_goal],  #TODO: path_planner, navigate_to_goal
            memory=False)
        
        path_planning_pick = py_trees.composites.Parallel(
            name = 'path_plan_pick', 
            children = [self.pub_occupancy_grid, self.localization, plan_and_move],
            policy = py_trees.common.ParallelPolicy.SuccessOnSelected([plan_and_move]))
        
        # Arm execution: pick or place
            # place
        place = Place()
            # Pick and lift operations
        planA = py_trees.composites.Sequence(
            name="PlanA", 
            children = [self.tuck_arm, self.pick_object],
            memory = False)
        pick_and_lift = py_trees.composites.Sequence(
            name="Pick&Lift", 
            children = [planA, self.lift],
            memory = False)
        repeat_picklift = py_trees.decorators.Retry(
            name = 'Repeat_Pick&Lift', 
            child = pick_and_lift, 
            num_failures = 2)
            # selector between them
        pick_or_place = py_trees.composites.Selector(
            name = 'Pick_or_Place', 
            children = [place, repeat_picklift],
            memory = False)
        
        self.root.add_children([self.next_object_bhv, path_planning_pick, pick_or_place, self.arm_task_succeeded]) 

        return self.root
    
    def create_lists(self):
        """ Function to return a list with the objects to collect, as well as the boxes"""
        data = []
        with open(self.filename, 'r') as f:
            for line in f:
                line = line.strip() # remove leading and trailing whitespaces
                words = line.split('\t')
                data.append(words)

        objs_list = [[sublist[0]] + [0.01*float(x) for x in sublist[1:]] for sublist in data if sublist[0] != 'B']
        box_list = [[sublist[0]] + [0.01*float(x) for x in sublist[1:]] for sublist in data if sublist[0] == 'B']

        return objs_list, box_list



def main(args=None):
    rclpy.init(args=args)
    node = CollectionBT()
    executor = MultiThreadedExecutor()
    root = node.create_root(executor=executor)
    node.tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    executor.add_node(node)
    node.tree.setup(timeout=10.0, node=node)
    time.sleep(2.0)
    node.tree.tick_tock(period_ms=400)

    try:
         executor.spin()
    except KeyboardInterrupt:
        pass  

    rclpy.shutdown()


if __name__ == '__main__':
    main()



# # ------------------------ 1st and 2nd steps of sequence
#         compute_next_object = I_NextObject()
#         plan_path = A_PlanPath()
#         move_to_pick = A_MoveRobot()
#         update_obj_list = I_UpdateObjectList()

#         plan_and_move.add_children([plan_path, move_to_pick])
#         path_planning_execution = py_trees.composites.Selector(name="PPE", memory= False)

#         plan_and_move = py_trees.composites.Sequence(name="Plan_and_Move", memory= False)
#         path_planning_execution.add_children([plan_and_move, update_obj_list])

#         # before 3rd step, define every hard-coded set of angles
#         arm_camera_tuck_angles = [2600,12000,12000,12000,12000,12000] 
#         lift_angles = [2600,12000,12000,12000,12000,12000] 

#         # ------------------------ 3rd step
#         pick_lift = py_trees.composites.Sequence(name="Pick&Lift", memory= False)

#         planA = py_trees.composites.Sequence(name="PlanA", memory= False)
#         planB = py_trees.composites.Parallel(name="PlanB", memory= False)

#         #plan A
#         camera_tuck = SetArm('camera_tuck', arm_camera_tuck_angles) 
#         arm_camera_detection = DetectObject()
#         move_arm_around = SearchObjectArm()
#         search_loop = py_trees.composites.Parallel(name="PlanB", 
#                     policy = py_trees.common.ParallelPolicy.SuccessOnOne, children = [arm_camera_detection, move_arm_around])
#         arm_ik = ArmIK()
#         planA.add_children([camera_tuck, search_loop, arm_ik])
#         #plan B TODO

#         pick_or_search = py_trees.composites.Selector(name="Pick&Lift", memory= False, children=[planA]) #put planB
#         # lift
#         lift = SetArm('lift', lift_angles) 

#         repeat_picklift = py_trees.decorators.Retry(
#             name = 'Repeat_Pick&Lift', 
#             child = py_trees.composites.Sequence([pick_or_search, lift]), 
#             num_failures = 5)

#         # ------------------------ 4th step


#         root.add_children([compute_next_object, path_planning_execution]) #, move_to_pick, obj_tuck_bhv