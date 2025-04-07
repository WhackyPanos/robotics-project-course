#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from handle_objects.pick_objects import SetArm, SearchObjectArm, ArmIK, Place
from behavior_tree.goCollect_bhv import goTo
from rclpy.executors import MultiThreadedExecutor
import os
from .collection_bhv  import UpdateObjectList, ArmTaskSucceeded, Adjust
from path_planner.motion_bhv_collection import NavigateToGoal
from localization.localization_bhv import Localization_bhv
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from detection_bt.arm_segmentation_bt import ArmSegmentationBT
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_planner.pathPlanning_bhv import PathPlan

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
  
        # root and behaviors creation
        self.root = py_trees.composites.Sequence(name="Root", memory= True)

        # self.place_tuck_arm = SetArm('place_tuck_arm', [1000,12000,12000,12000,8000,12000], 500)
        # self.place_open_gripper = SetArm('place_open_gripper', [2600,12000,12000,12000,8000,12000], 1000)
        # self.place_lift = SetArm('place_lift', [2600,12000,12000,12000,12000,12000], 200)

        self.tuck_arm = SetArm('tuck_arm', [2600,12000,2000,20000,12000,12000], 200)
        self.detect_object = ArmSegmentationBT()
        self.pick_object = ArmIK()
        self.lift = SetArm('lift', [10000,12000,12000,12000,12000,12000], 200)
        self.adjust = Adjust()
        self.place = Place()
        self.path_plan = PathPlan()
        
        #self.pub_occupancy_grid = PublishOccupancyGrid()
        #self.localization = Localization_bhv()
        self.navigate_to_goal = NavigateToGoal()
        self.next_object_bhv = UpdateObjectList(self.objs_list, self.box_list, "next_object")
        self.path_planner = None #TODO

        self.arm_task_succeeded = ArmTaskSucceeded()

    def create_executor(self, executor):
        # Add nodes to executor
        executor.add_node(self.tuck_arm)
        executor.add_node(self.pick_object)
        executor.add_node(self.lift)
        executor.add_node(self.detect_object)
        executor.add_node(self.arm_task_succeeded)
        executor.add_node(self.adjust)
        executor.add_node(self.place)
    
        executor.add_node(self.next_object_bhv)
        executor.add_node(self.navigate_to_goal)
        #executor.add_node(self.pub_occupancy_grid)
        #executor.add_node(self.localization)


        #executor.add_node(self.pub_occupancy_grid.occupancy_grid)
        executor.add_node(self.navigate_to_goal.motion_node)
        #executor.add_node(self.localizatio
        # n.localization_node)


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
        self.pick_or_adjust = py_trees.composites.Selector(
            name = 'Pick_or_Adjust', 
            children = [self.pick_object, self.adjust], #TODO: if working fine, make this sequence with adjust first
            memory = True)
        planA = py_trees.composites.Sequence(
            name="PlanA", 
            children = [self.tuck_arm, py_trees.timers.Timer("Timer", duration=5), self.detect_object, self.pick_or_adjust],
            memory = True)
        # self.pick_and_lift = py_trees.composites.Sequence(
        #     name="Pick&Lift", 
        #     children = [planA, self.lift],
        #     memory = True)
        # self.repeat_picklift = py_trees.decorators.Retry(
        #     name = 'Repeat_Pick&Lift', 
        #     child = self.pick_and_lift, 
        #     num_failures = 2)
            # selector between them
        self.pick_or_place = py_trees.composites.Selector(
            name = 'Pick_or_Place', 
            children = [self.place,planA], # self.repeat_picklift
            memory = False)

        self.root.add_children([self.next_object_bhv, self.path_plan, self.navigate_to_goal, self.pick_or_place, self.lift, self.arm_task_succeeded])
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
        return objs_list, box_list
    
    def publish_initial_transform(self):
        t = TransformStamped()
        t.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        self.get_logger().info('Publishing initial map to odom frame (in collection_bt node)')
        self.tf_broadcaster.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    node = CollectionBT() # behaviors creation
    executor = MultiThreadedExecutor()
    node.create_executor(executor=executor)
    node.create_tree()
    executor.add_node(node)
    node.tree.setup(timeout=10.0, node=node)
    time.sleep(2.0)
    node.tree.tick_tock(period_ms=250)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown() 

    #rclpy.shutdown()


if __name__ == '__main__':
    main()


""" File with motion node"""

# #!/usr/bin/env python
# import rclpy
# import py_trees
# import py_trees_ros
# from rclpy.node import Node
# from py_trees_ros.trees import BehaviourTree
# from handle_objects.pick_objects import SetArm, SearchObjectArm, ArmIK, Place
# from behavior_tree.goCollect_bhv import goTo
# from rclpy.executors import MultiThreadedExecutor
# import os
# from .collection_bhv  import UpdateObjectList, ArmTaskSucceeded, Adjust
# from path_planner.motion_bhv import NavigateToGoal
# from localization.localization_bhv import Localization_bhv
# from mapping.PublishOccupancyGrid_bhv import PublishOccupancyGrid
# import time
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# from detection_bt.arm_segmentation_bt import ArmSegmentationBT



# class CollectionBT(Node):
#     def __init__(self) -> None:
#         super().__init__('behavior_tree')
#         relative_path_to_file = os.path.join("/home/group3-robot/robp_group3/robp_ws/src/behavior_tree", "map_1.tsv")
#         self.filename = os.path.realpath(relative_path_to_file) #introduce name of the text file
#         self.objs_list, self.box_list = self.create_lists()
#         self.get_logger().info(f"Bla {self.objs_list, self.box_list}")
#         self.tf_broadcaster = TransformBroadcaster(self)
#         #self.publish_initial_transform()
  
#         # root and behaviors creation
#         self.root = py_trees.composites.Sequence(name="Root", memory= True)

#         # self.place_tuck_arm = SetArm('place_tuck_arm', [1000,12000,12000,12000,8000,12000], 500)
#         # self.place_open_gripper = SetArm('place_open_gripper', [2600,12000,12000,12000,8000,12000], 1000)
#         # self.place_lift = SetArm('place_lift', [2600,12000,12000,12000,12000,12000], 200)

#         self.tuck_arm = SetArm('tuck_arm', [2600,12000,2000,20000,12000,12000], 200)
#         self.detect_object = ArmSegmentationBT()
#         self.pick_object = ArmIK()
#         self.lift = SetArm('lift', [10000,12000,12000,12000,12000,12000], 200)
#         self.adjust = Adjust()
#         self.place = Place()
        
#         #self.pub_occupancy_grid = PublishOccupancyGrid()
#         #self.localization = Localization_bhv()
#         self.navigate_to_goal = NavigateToGoal()
#         self.next_object_bhv = UpdateObjectList(self.objs_list, self.box_list, "next_object")
#         self.path_planner = None #TODO

#         self.arm_task_succeeded = ArmTaskSucceeded()

#     def create_executor(self, executor):
#         # Add nodes to executor
#         executor.add_node(self.tuck_arm)
#         executor.add_node(self.pick_object)
#         executor.add_node(self.lift)
#         executor.add_node(self.detect_object)
#         executor.add_node(self.arm_task_succeeded)
#         executor.add_node(self.adjust)
#         executor.add_node(self.place)
    
#         executor.add_node(self.next_object_bhv)
#         executor.add_node(self.navigate_to_goal)
#         #executor.add_node(self.pub_occupancy_grid)
#         #executor.add_node(self.localization)


#         #executor.add_node(self.pub_occupancy_grid.occupancy_grid)
#         executor.add_node(self.navigate_to_goal.motion_node)
#         #executor.add_node(self.localizatio
#         # n.localization_node)


#     def create_tree(self): 
#         """ merge behaviors with composites """
#         # Path planning and execution for picking
#         plan_and_move = py_trees.composites.Sequence(
#             name = 'plan_and_move',
#             children = [self.navigate_to_goal],  #TODO: path_planner, navigate_to_goal
#             memory=False)
        
#         self.path_planning_pick = py_trees.composites.Parallel(
#             name = 'path_plan_pick', 
#             children = [plan_and_move], #self.localization, self.pub_occupancy_grid, 
#             policy = py_trees.common.ParallelPolicy.SuccessOnSelected([plan_and_move]))
        
#         # Arm execution: pick or place
#             # Pick and lift operations
#         self.pick_or_adjust = py_trees.composites.Selector(
#             name = 'Pick_or_Adjust', 
#             children = [self.pick_object, self.adjust], #TODO: if working fine, make this sequence with adjust first
#             memory = True)
#         planA = py_trees.composites.Sequence(
#             name="PlanA", 
#             children = [self.tuck_arm, self.detect_object, self.pick_or_adjust],
#             memory = True)
#         # self.pick_and_lift = py_trees.composites.Sequence(
#         #     name="Pick&Lift", 
#         #     children = [planA, self.lift],
#         #     memory = True)
#         # self.repeat_picklift = py_trees.decorators.Retry(
#         #     name = 'Repeat_Pick&Lift', 
#         #     child = self.pick_and_lift, 
#         #     num_failures = 2)
#             # selector between them
#         self.pick_or_place = py_trees.composites.Selector(
#             name = 'Pick_or_Place', 
#             children = [self.place,planA], # self.repeat_picklift
#             memory = False)

#         self.root.add_children([self.next_object_bhv, self.path_planning_pick, self.pick_or_place, self.lift, self.arm_task_succeeded])
#         self.tree = py_trees_ros.trees.BehaviourTree(root=self.root, unicode_tree_debug=False) 

#         return 
    
#     def create_lists(self):
#         """ Function to return a list with the objects to collect, as well as the boxes"""
#         data = []
#         with open(self.filename, 'r') as f:
#             for line in f:
#                 line = line.strip() # remove leading and trailing whitespaces
#                 words = line.split('\t')
#                 data.append(words)

#         objs_list = [[sublist[0]] + [0.01*float(x) for x in sublist[1:]] for sublist in data if sublist[0] != 'B']
#         box_list = [[sublist[0]] + [0.01*float(x) for x in sublist[1:]] for sublist in data if sublist[0] == 'B']

#         return objs_list, box_list
    
#     def publish_initial_transform(self):
#         t = TransformStamped()
#         t.header.stamp = rclpy.time.Time(seconds=0).to_msg()
#         t.header.frame_id = 'map'
#         t.child_frame_id = 'odom'
#         self.get_logger().info('Publishing initial map to odom frame (in collection_bt node)')
#         self.tf_broadcaster.sendTransform(t)



# def main(args=None):
#     rclpy.init(args=args)
#     node = CollectionBT() # behaviors creation
#     executor = MultiThreadedExecutor()
#     node.create_executor(executor=executor)
#     node.create_tree()
#     executor.add_node(node)
#     node.tree.setup(timeout=10.0, node=node)
#     time.sleep(2.0)
#     node.tree.tick_tock(period_ms=100)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         rclpy.shutdown() 

#     #rclpy.shutdown()


# if __name__ == '__main__':
#     main()

	
