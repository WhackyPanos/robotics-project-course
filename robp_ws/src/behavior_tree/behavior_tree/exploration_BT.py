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
        executor.add_node(self.obstacle_on_path_detected)

        executor.add_node(self.update_map_file.map_file_node)
        executor.add_node(self.pub_occupancy_grid.occupancy_grid)
        executor.add_node(self.navigate_to_goal.motion_node)
        executor.add_node(self.goal.random_point_node)
        executor.add_node(self.path_plan.path_planner)
        executor.add_node(self.obstacle_on_path_detected.check_path)
        

        fourth_sequence = py_trees.composites.Sequence(name='third_seq', memory=True)
        fourth_sequence.add_children([self.object_detected, self.classify, self.update_map_file])

        short_wait = py_trees.timers.Timer("Timer", duration=5)

        decorator = py_trees.decorators.Repeat(
            name='dec_repeat', 
            child=fourth_sequence,
            num_success=5   # 5 consecutive successes
        )

        # second_sequence = py_trees.composites.Sequence(name='second_seq', memory=True)
        # second_sequence.add_children([self.new_object_detected, decorator])

        # Parallel Node: Runs both detection and navigation simultaneously, success on one
        second_parallel = py_trees.composites.Parallel(
            name="parallel_detect_navigate",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )
        second_parallel.add_children([self.new_object_detected, self.obstacle_on_path_detected, self.navigate_to_goal])

        third_sequence = py_trees.composites.Sequence(name='third_seq', memory=True)
        third_sequence.add_children([short_wait, decorator])

        # EternalGuard: Ensures that the decorator only runs if new_object_detected is successful
        object_detected_guard = py_trees.decorators.EternalGuard(
            name="object_detected_guard", 
            child=third_sequence,                     # The decorator should only execute if the condition is met
            condition=self.object_detected_condition  # Condition to check if new_object_detected was successful
        )

        fail_is_success = py_trees.decorators.FailureIsSuccess(name='fail2success', child=object_detected_guard)

        second_sequence = py_trees.composites.Sequence(name="second_seq", memory=True)
        second_sequence.add_children([second_parallel, fail_is_success])

        first_sequence = py_trees.composites.Sequence(name='first_seq', memory=True)
        first_sequence.add_children([self.goal, self.path_plan, second_sequence, self.pub_occupancy_grid])

        timer = CustomTimer(name='timer', duration=300.0)

        first_parallel = py_trees.composites.Parallel(
            name="Stop Tree Parallel",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )

        first_parallel.add_children([timer, self.check_occupancy_grid])

        # Add behavior tree child nodes to the root
        self.root.add_children([first_parallel, first_sequence])

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
    time.sleep(3.0)

    # Continuously tick the behavior tree
    node.tree.tick_tock(period_ms=10)

    # Continuously tick the behavior tree
    try:
         executor.spin()
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully

    # Shutdown ROS2 and the executor
    rclpy.shutdown()

if __name__ == '__main__':
    main()


