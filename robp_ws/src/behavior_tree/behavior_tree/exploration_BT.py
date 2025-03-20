#!/usr/bin/env python
import rclpy
import py_trees
import py_trees_ros
from rclpy.node import Node
from py_trees_ros.trees import BehaviourTree

# Import classes
# from exploration_bhv import UnexploredMap, PathPlan 
from path_planner.motion_bhv import NavigateToGoal
from obstacle_on_path.obstacle_on_path_bhv  import ObstacleOnPath
from detection_bt.classify_bt import ClassifyBT
from detection_bt.cluster_bt import ClusterBT
from map_file.map_file_bt import MapFileBT
from mapping.PublishOccupancyGrid_bhv import PublishOccupancyGrid

class ExplorationBT(Node):
    def __init__(self) -> None:
        super().__init__('behavior_tree_exploration')
        root = self.create_root()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)

    def create_root(self):    
        # Create the root as a Sequence node (default memory=False is fine here)
        root = py_trees.composites.Sequence(name="Root", memory= False)

        # map_not_fully_explored = UnexploredMap()
        # path_plan = PathPlan()
        navigate_to_goal = NavigateToGoal()
        pub_occupancy_grid = PublishOccupancyGrid()
        obstacle_on_path_detected = ObstacleOnPath()
        classify = ClassifyBT()
        new_object_detected = ClusterBT()
        update_map_file = MapFileBT()

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

        root.add_children([navigate_to_goal, pub_occupancy_grid])
        
        return root



def main(args=None):
    rclpy.init(args=args)

    # Initialize the ExplorationBT node
    node = ExplorationBT()

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