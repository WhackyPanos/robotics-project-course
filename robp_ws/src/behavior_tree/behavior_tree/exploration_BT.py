#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree

# Import classes
from path_planner.random_point_bhv import PointGenerator
from path_planner.motion_bhv import NavigateToGoal
from obstacle_on_path.obstacle_on_path_bhv  import ObstacleOnPath
from detection_bt.classify_bt import ClassifyBT
from detection_bt.cluster_bt import ClusterBT
from map_file.map_file_bt import MapFileBT
from mapping.PublishOccupancyGrid_bhv import PublishOccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from localization.localization_transform import Localization

class ExplorationBT(Node):
    def __init__(self) -> None:
        super().__init__('behavior_tree_exploration')
        # Create the root as a Sequence node
        self.root = py_trees.composites.Sequence(name="Root", memory=True)

        self.goal = PointGenerator()
        self.navigate_to_goal = NavigateToGoal()
        self.pub_occupancy_grid = PublishOccupancyGrid()
        self.obstacle_on_path_detected = ObstacleOnPath()
        self.classify = ClassifyBT()
        self.new_object_detected = ClusterBT()
        self.update_map_file = MapFileBT()
        self.tree = py_trees_ros.trees.BehaviourTree(root=self.root, unicode_tree_debug=False)

    def create_root(self, executor):
        """
        Create the behavior tree root and add children nodes
        """
        # Add nodes to executor
        executor.add_node(self.goal)
        executor.add_node(self.navigate_to_goal)
        executor.add_node(self.pub_occupancy_grid)
        executor.add_node(self.pub_occupancy_grid.occupancy_grid)
        executor.add_node(self.navigate_to_goal.motion_node)
        executor.add_node(self.goal.random_point_node)

        # Add behavior tree child nodes to the root
        self.root.add_children([self.pub_occupancy_grid, self.goal, self.navigate_to_goal])

        return self.root

        # second_sequence = py_trees.composites.Sequence(name='second_seq')
        # second_sequence.add_children([new_object_detected, classify, update_map_file])

        # second_selector = py_trees.composites.Selector(name='second_sel')
        # second_selector.add_children([obstacle_on_path_detected, second_sequence])

        # first_parallel = py_trees.composites.Parallel(name='first_parallel')
        # first_parallel.add_children([second_selector, update_occupnacy_grid])

        # first_selector = py_trees.composites.Selector(name='first_sel')
        # first_selector.add_children([first_parallel, navigate_to_goal])

        # first_sequence = py_trees.composites.Sequence(name='first_seq')
        # first_sequence.add_children([path_plan, first_selector])

        # root.add_children([pub_occupancy_grid, goal, navigate_to_goal])
        
        # return root



def main(args=None):
    rclpy.init(args=args)

    # Initialize the executor with a valid context
    executor = MultiThreadedExecutor()

    # Initialize the ExplorationBT node
    node = ExplorationBT()

    # Create the root and set up the behavior tree
    root = node.create_root(executor=executor)
    tree = py_trees_ros.trees.BehaviourTree(root=root)

    # Add the node to the executor
    executor.add_node(node)

    # Setup the behavior tree with a timeout for setup (10 seconds)
    node.tree.setup(timeout=10.0, node=node)

    # Continuously tick the behavior tree
    node.tree.tick_tock(period_ms=300)

    # Continuously tick the behavior tree
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=1.0)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully

    # Shutdown ROS2 and the executor
    rclpy.shutdown()

if __name__ == '__main__':
    main()


