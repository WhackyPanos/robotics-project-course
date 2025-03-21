#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree
from rclpy.executors import MultiThreadedExecutor
import time

# Import classes
from .customTimer_BT import CustomTimer
from path_planner.random_point_bhv import PointGenerator
from path_planner.motion_bhv import NavigateToGoal
from obstacle_on_path.obstacle_on_path_bhv  import ObstacleOnPath
from detection_bt.classify_bt import ClassifyBT
from detection_bt.cluster_bt import ClusterBT
from map_file.map_file_bt import MapFileBT
from path_planner.pathPlanning_bhv import PathPlan
from mapping.PublishOccupancyGrid_bhv import PublishOccupancyGrid
from mapping.CheckUnexploredSpace_bhv import CheckOccupancyGrid
# from localization.localization_transform import Localization

class ExplorationBT(Node):
    def __init__(self) -> None:
        super().__init__('behavior_tree_exploration')
        # Create the root
        self.root  = py_trees.composites.Selector(name='tree_sel', memory=False)

        self.tree = py_trees_ros.trees.BehaviourTree(root=self.root, unicode_tree_debug=False)

        self.goal = PointGenerator()
        self.navigate_to_goal = NavigateToGoal()
        self.pub_occupancy_grid = PublishOccupancyGrid()
        self.check_occupancy_grid = CheckOccupancyGrid()
        self.obstacle_on_path_detected = ObstacleOnPath()
        self.new_object_detected = ClusterBT(new_request=True)
        self.object_detected = ClusterBT(new_request=False)
        self.classify = ClassifyBT()
        self.update_map_file = MapFileBT()
        self.tree = py_trees_ros.trees.BehaviourTree(root=self.root, unicode_tree_debug=False)
        self.path_plan = PathPlan()

    def create_root(self, executor):
        """
        Create the behavior tree root and add children nodes
        """
        # Add nodes to executor
        executor.add_node(self.goal)
        executor.add_node(self.navigate_to_goal)
        executor.add_node(self.pub_occupancy_grid)
        executor.add_node(self.check_occupancy_grid)
        executor.add_node(self.new_object_detected)
        executor.add_node(self.object_detected)
        executor.add_node(self.classify)
        executor.add_node(self.update_map_file)
        executor.add_node(self.path_plan)

        executor.add_node(self.update_map_file.map_file_node)
        executor.add_node(self.pub_occupancy_grid.occupancy_grid)
        executor.add_node(self.navigate_to_goal.motion_node)
        executor.add_node(self.goal.random_point_node)
        executor.add_node(self.path_plan.path_planner)
        executor.add_node(self.obstacle_on_path_detected.check_path)
        

        # third_sequence = py_trees.composites.Sequence(name='third_seq', memory=True)
        # third_sequence.add_children([self.object_detected, self.classify, self.update_map_file])

        # decorator = py_trees.decorators.Repeat(
        #     name='dec_repeat', 
        #     child=third_sequence,
        #     num_success=5   # 5 consecutive successes
        # )

        # second_sequence = py_trees.composites.Sequence(name='second_seq', memory=True)
        # second_sequence.add_children([self.new_object_detected, decorator])

        # first_selector = py_trees.composites.Selector(name='first_sel', memory=False)
        # first_selector.add_children([second_sequence, self.navigate_to_goal])

        # Add behavior tree child nodes to the root
        self.root.add_children([self.pub_occupancy_grid, self.goal, self.path_plan, self.navigate_to_goal])

        return self.root
    
    
    # Check for eternal guard if the new_object_detected behavior has succeeded
    def object_detected_condition(self):
        return self.new_object_detected.status == py_trees.common.Status.SUCCESS



def main(args=None):
    rclpy.init(args=args)

    # Initialize the executor with a valid context
    executor = MultiThreadedExecutor()

    # Initialize the ExplorationBT node
    node = ExplorationBT()

    # Create the root and set up the behavior tree
    node.create_root(executor=executor)

    # Add the node to the executor
    executor.add_node(node)

    # Setup the behavior tree with a timeout for setup (10 seconds)
    node.tree.setup(timeout=10.0, node=node)
    time.sleep(5.0)

    # Continuously tick the behavior tree
    node.tree.tick_tock(period_ms=50)

    # Continuously tick the behavior tree
    try:
         executor.spin()
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully

    # Shutdown ROS2 and the executor
    rclpy.shutdown()

if __name__ == '__main__':
    main()


