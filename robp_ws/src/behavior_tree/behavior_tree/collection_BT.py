#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import os
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from .customTimer_BT import CustomTimer
from detection_bt.arm_segmentation_bt import ArmSegmentationBT
from detection_bt.cluster_collection_bt import ClusterBT
from detection_bt.box_segmentation_bt import BoxSegmentationBT
from handle_objects.pick_objects import SetArm, SearchObjectArm, ArmIK, Place
from behavior_tree.goCollect_bhv import goTo
from .collection_bhv  import UpdateObjectList, ArmTaskSucceeded, Adjust
from path_planner.motion_bhv_collection import NavigateToGoal
from path_planner.revert_bhv import Revert
from path_planner.rotate_bhv import Rotate
from localization.localization_bhv import Localization_bhv
from path_planner.pathPlanning_bhv_collection import PathPlan
from obstacle_on_path.obstacle_on_path_bhv import ObstacleOnPath

class CollectionBT(Node):
    def __init__(self) -> None:

        super().__init__('behavior_tree')
        relative_path_to_file = os.path.join("/home/group3-robot/robp_group3/robp_ws/src/behavior_tree", "map_1.tsv")
        self.filename = os.path.realpath(relative_path_to_file) #introduce name of the text file
        self.object_path_publisher = self.create_publisher(Path, '/object_path', 10)
        self.objs_list, self.box_list = self.create_lists()
        #self.get_logger().info(f"Bla {self.objs_list, self.box_list}")
        self.tf_broadcaster = TransformBroadcaster(self)
        #self.publish_initial_transform()
        self.tick_tock = True

        # -------------------- Behaviors Parameters ------------------------------
        self.detect_and_adjust_num_reps = 4
        self.pick_and_lift_num_reps = 5
        # ----------------------------------------------------------------------
  
        # root and behaviors creation
        self.root = py_trees.composites.Selector(name="Root", memory= False)

        self.tuck_arm = SetArm('tuck_arm', [2600,12000,2000,20000,12000,12000], 200)
        #self.first_detect_object = ArmSegmentationBT(name="first_arm_segmentation")
        self.detect_object = ArmSegmentationBT()
        self.final_detect_object = ArmSegmentationBT(name = "final_arm_segmentation_bt")
        self.new_object_detected = ClusterBT(new_request=True)
        self.object_detected = ClusterBT(new_request=False)
        self.detect_box = BoxSegmentationBT()
        self.pick_object = ArmIK()
        self.lift = SetArm('lift', [10000,12000,12000,12000,12000,12000], 200)
        self.lift_after_placing = SetArm('lift_after_placing', [10000,12000,12000,12000,12000,12000], 200)
        self.adjust = Adjust()
        self.place = Place()
        self.path_plan = PathPlan()
        self.obstacle_on_path = ObstacleOnPath()
        
        #self.localization = Localization_bhv()
        self.navigate_to_goal = NavigateToGoal()
        self.revert = Revert()
        self.rotate = Rotate()
        self.next_object_bhv = UpdateObjectList(self.objs_list, self.box_list, "next_object")

    def create_tree(self): 
        """ merge behaviors with composites """
        # Path planning and execution for picking
        # plan_and_move = py_trees.composites.Sequence(
        #     name = 'plan_and_move',
        #     children = [self.navigate_to_goal],  #TODO: path_planner, navigate_to_goal
        #     memory=False)
        
        # self.path_planning_pick = py_trees.composites.Parallel(
        #     name = 'path_plan_pick', 
        #     children = [plan_and_move], #self.localization, self.pub_occupancy_grid, 
        #     policy = py_trees.common.ParallelPolicy.SuccessOnSelected([plan_and_move]))
        
        # Arm execution: pick or place
            # Pick and lift operations
        detect_and_adjust = py_trees.composites.Sequence( 
            name = 'Detect_and_Adjust', 
            children = [py_trees.timers.Timer("Timer", duration=2), self.detect_object, py_trees.timers.Timer("Timer", duration=1), self.adjust], #TODO: if working fine, make this sequence with adjust first
            memory = True)       
        repeat_detect_and_adjust = py_trees.decorators.Retry( # TODO introduce selector between retry and a behavior that return success, if needed
            name = 'Repeat_Detect&Adjust', 
            child = detect_and_adjust, 
            num_failures = self.detect_and_adjust_num_reps)
        fail_is_sucess =  py_trees.decorators.FailureIsSuccess(
            name = " Fail is Success (Repeat_Detect&Adjust)",
            child = repeat_detect_and_adjust
        )
        planA = py_trees.composites.Sequence( # removed self.first_detect_object, 
            name="PlanA", 
            children = [self.tuck_arm, py_trees.timers.Timer("Timer", duration=2), fail_is_sucess, self.final_detect_object, py_trees.timers.Timer("Timer", duration=0.5), self.pick_object],
            memory = True)
        
        planA_or_rotate = py_trees.trees.composites.Selector(
            name='PlanA or Rotate',
            memory=True,
            children=[planA, self.rotate]
        )

        pick_and_lift = py_trees.composites.Sequence(
            name="Pick&Lift", 
            children = [planA_or_rotate, self.lift, py_trees.timers.Timer("Timer", duration=2)],
            memory = True)
        
        repeat_picklift = py_trees.decorators.Retry(
            name = 'Repeat_Pick&Lift', 
            child = pick_and_lift, 
            num_failures = self.pick_and_lift_num_reps)
        
        return_fail_repeat_picklift = py_trees.decorators.SuccessIsFailure(
            name = "Success is fail (Repeat_Pick&Lift)",
            child = repeat_picklift
        )

        nav_and_check = py_trees.composites.Parallel(
            name = 'Nav and Check Path',
            policy = py_trees.common.ParallelPolicy.SuccessOnOne(),
            children= [self.obstacle_on_path, self.new_object_detected, self.navigate_to_goal]
        )

        retry_cluster = py_trees.decorators.Retry(
            name='Retry Cluster',
            child=self.object_detected,
            num_failures=10
        )

        update_goal = py_trees.composites.Sequence(
            name='Update goal pos',
            memory=True,
            children=[py_trees.timers.Timer(name='Timer Cluster', duration=2), retry_cluster]
        )
        
        # EternalGuard: Ensures that the decorator only runs if new_object_detected is successful
        object_detected_guard = py_trees.decorators.EternalGuard(
            name="Object Detect Guard", 
            child=update_goal,                     
            condition=self.object_detected_condition
        )

        nav_or_update_goal = py_trees.composites.Selector(
            name='Nav or Update goal',
            memory=True,
            children=[nav_and_check, object_detected_guard] #object_detected_guard
        )

        nav_and_check_seq = py_trees.composites.Sequence(
            name='Plan and Nav',
            children=[self.path_plan, nav_or_update_goal],
            memory=True
        )

        repeat_navigate = py_trees.decorators.Retry(
            name = 'Repeat Plan and Nav',
            child=nav_and_check_seq,
            num_failures= 30000
        )

        place_and_revert = py_trees.composites.Sequence(
            name='Place and Revert',
            children=[self.place, py_trees.timers.Timer(name='place_timer', duration=1), self.lift_after_placing, py_trees.timers.Timer(name='place_timer', duration=1), self.revert],
            memory=True
        )

        pick_or_place = py_trees.composites.Selector(
            name = 'Pick_or_Place', 
            children = [place_and_revert, return_fail_repeat_picklift], # self.repeat_picklift
            memory = False)
        
        main_sequence = py_trees.composites.Sequence(
            name = 'Collection bhv',
            children = [self.next_object_bhv, py_trees.timers.Timer(name='place_timer', duration=1), repeat_navigate,  pick_or_place], #NOTE: add/remove repeat_navigate
            memory = True
        )

        self.root.add_children([CustomTimer(name='timer', duration=300.0), main_sequence])
        self.tree = py_trees_ros.trees.BehaviourTree(root=self.root, unicode_tree_debug=False) 

        return 
    
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

        full_list = objs_list + box_list  # Combine both for a single path

        object_path = Path()
        object_path.header.frame_id = "map"  # Global frame to tie it all together

        for obj in full_list:
            label, x, y = obj[0], obj[1], obj[2]
            pose = PoseStamped()
            pose.header.frame_id = label  # Use object label as frame ID
            pose.pose.position.x = x
            pose.pose.position.y = y
            object_path.poses.append(pose)
        
        self.object_path_publisher.publish(object_path)
        self.get_logger().info('Publishing object path')
        return objs_list, box_list
    
    def publish_initial_transform(self):
        t = TransformStamped()
        t.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        self.get_logger().info('Publishing initial map to odom frame (in collection_bt node)')
        self.tf_broadcaster.sendTransform(t)

    # Check for eternal guard if the new_object_detected behavior has succeeded
    def object_detected_condition(self):
        # self.get_logger().info(f"Detect : {self.new_object_detected.status}")
        # self.get_logger().info(f"Obstacle on Path Status: {self.obstacle_on_path.status}")
        # self.get_logger().info(f"Condition: {self.obstacle_on_path.status == py_trees.common.Status.INVALID}")
        # return self.new_object_detected == py_trees.common.Status.FAILURE
        return self.obstacle_on_path.status == py_trees.common.Status.INVALID
    
    def init_tree_callback(self):
        if self.tick_tock:
            self.tree.tick_tock(period_ms=10)
        self.tick_tock = False

    def create_executor(self, executor, executor_single):
        # Add nodes to multi threaded executor
        executor.add_node(self.tuck_arm)
        executor.add_node(self.pick_object)
        executor.add_node(self.lift)
        executor.add_node(self.lift_after_placing)
        executor.add_node(self.detect_object)
        executor.add_node(self.final_detect_object)
        executor.add_node(self.object_detected)
        executor.add_node(self.new_object_detected)
        executor.add_node(self.detect_box)
        executor.add_node(self.adjust)
        executor.add_node(self.place)
    
        executor.add_node(self.next_object_bhv)
        executor.add_node(self.navigate_to_goal)
        executor.add_node(self.revert)
        executor.add_node(self.rotate)
        executor.add_node(self.path_plan)
        executor.add_node(self.obstacle_on_path)


        executor.add_node(self.navigate_to_goal.motion_node)
        executor.add_node(self.revert.motion_node)
        executor.add_node(self.rotate.motion_node)
        executor.add_node(self.path_plan.path_planner)
        executor.add_node(self.obstacle_on_path.check_path)



        # Add nodes to single threaded executor #TODO: incorporate main camera nodes in executors to avoid blocking events
        # executor_single.add_node(self.place)

        #executor_single.add_node(self.next_object_bhv)
        # executor_single.add_node(self.object_detected)
        # executor_single.add_node(self.new_object_detected)
        # executor_single.add_node(self.detect_box)

        # executor_single.add_node(self.navigate_to_goal)
        # executor_single.add_node(self.revert)
        # executor_single.add_node(self.rotate)

        # executor_single.add_node(self.navigate_to_goal.motion_node)
        # executor_single.add_node(self.revert.motion_node)
        # executor_single.add_node(self.rotate.motion_node)


        


def main(args=None):
    rclpy.init(args=args)
    node = CollectionBT() # behaviors creation
    executor = MultiThreadedExecutor()
    executor_single = SingleThreadedExecutor()
    node.create_executor(executor=executor, executor_single=executor_single)
    node.create_tree()
    executor.add_node(node)
    node.tree.setup(timeout=10.0, node=node)
    # time.sleep(2.0)
    # node.tree.tick_tock(period_ms=500)
    node.create_timer(10,node.init_tree_callback)
    node.get_logger().info('The tree will be ticked in a few seconds')

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)
            #executor_single.spin_once(timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown() 

    #rclpy.shutdown()


if __name__ == '__main__':
    main()

