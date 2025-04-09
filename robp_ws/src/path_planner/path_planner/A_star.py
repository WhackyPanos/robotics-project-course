import rclpy
import math
import numpy as np
import heapq
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
#from scipy.ndimage import binary_dilation
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
#from mapping.occupancy_grid import OccupancyGridNode

class Nodes:
        def __init__(self, x, y):
            self.x = x # Observe that these are occupancy grid indexes
            self.y = y
            self.g = None
            self.h = None
            self.f = None
            self.parent = None

        def __lt__(self, other):
            return self.f < other.f

        def is_ok(self, config_space):
            # Check if within map bounds
            if (self.x < 0 or self.x >= config_space.info.width or 
                self.y < 0 or self.y >= config_space.info.height):
                return False
            
            if self.parent == None: # All children of the start node will be ok to avoid getting stuck when starting
                 print('Node has no parent')
                 return True

            config_space_data = np.array(config_space.data).reshape((config_space.info.height, config_space.info.width))
            
            # Check if in free space or unknown (0) rather than occupied (100)
            return config_space_data[self.y][self.x] == 0

        def get_children(self, node_goal, config_space, cost_ratio):
            ok_children_list = []
            step_size = 1
            directions = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]

            children_list = [Nodes(self.x + dx * step_size, self.y + dy * step_size) for dx, dy in directions]
            for node_child in children_list:
                node_child.parent = self
                if node_child.is_ok(config_space):
                    node_child.g = self.g + ((self.x - node_child.x)**2 + (self.y - node_child.y)**2)**0.5 
                    node_child.h = ((node_goal.x - node_child.x)**2 + (node_goal.y - node_child.y)**2)**0.5
                    node_child.f = cost_ratio*node_child.h + node_child.g
                    ok_children_list.append(node_child)
            return ok_children_list

class Planner_A_star(Node):
    def __init__(self):
        super().__init__('planner_A_star')
        self.simple_publisher = self.create_publisher(Path ,'/motion/path' , 10)
        self.full_publisher = self.create_publisher(Path ,'/full_path' , 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/config_space', self.map_callback, 10)
        self.goal_subscription = self.create_subscription(PointStamped, '/goal_point', self.goal_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) 

        # Initizialize
        # self.robot_radius = 0.20
        self.cost_ratio = 1
        self.config_space = None
        self.goal_msg = None
    
    def goal_callback(self, msg):
        self.goal_msg = msg
        
    def map_callback(self, config_space_msg): # Create the configurations space
        self.config_space = config_space_msg

    def path_plan(self, goal_threash=None): # Called from the behavior   
        # Get current position
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))  # I want the latest possible transform      
            start_x = t.transform.translation.x
            start_y = t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Lookup failed: {str(e)}")
            return False
        if self.config_space is None:
            self.get_logger().warn('Configuration space not recived for path planning')
            return False
        if self.goal_msg is None:
            self.get_logger().warn('Goal point not recived for path planning')
            return False

        # Convert world to grid
        i_start_x, i_start_y = self.world_to_grid(start_x, start_y)
        i_goal_x, i_goal_y = self.world_to_grid(self.goal_msg.point.x, self.goal_msg.point.y)

        node_goal = Nodes(i_goal_x, i_goal_y)
        node_start = Nodes(i_start_x, i_start_y)
        node_start.g = 0
        node_start.h = ((i_goal_x - i_start_x)**2 + (i_goal_y - i_start_y)**2)**0.5
        node_start.f = self.cost_ratio*node_start.h + node_start.g
        
        # Publish path
        simplified_path, full_path = self.a_star(node_start, node_goal, goal_threash)
        if full_path is None: return False

        self.simple_publisher.publish(simplified_path)
        self.full_publisher.publish(full_path)

        return True
        
    def construct_path(self, node_current):
        node_list = [node_current]
        while node_current.parent != None:
            node_list.append(node_current.parent)
            node_current = node_current.parent
        node_list.reverse()
        
        full_path_msg = Path()
        full_path_msg.header.frame_id = "map"
        full_path_msg.header.stamp = self.get_clock().now().to_msg()

        simplified_path_msg = Path()  # Create a Path message
        simplified_path_msg.header.frame_id = "map"
        simplified_path_msg.header.stamp = self.get_clock().now().to_msg()

        # Ensure first pose is added to simplified path
        first_pose = PoseStamped()
        first_pose.header.frame_id = "map"
        first_pose.header.stamp = self.get_clock().now().to_msg()
        first_pose.pose.position.x = node_list[0].x * self.config_space.info.resolution + self.config_space.info.origin.position.x
        first_pose.pose.position.y = node_list[0].y * self.config_space.info.resolution + self.config_space.info.origin.position.y
        full_path_msg.poses.append(first_pose)
        # simplified_path_msg.poses.append(first_pose)

        for i in range(1, len(node_list) - 1):  
            node = node_list[i]
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = node.x * self.config_space.info.resolution + self.config_space.info.origin.position.x
            pose.pose.position.y = node.y * self.config_space.info.resolution + self.config_space.info.origin.position.y
            full_path_msg.poses.append(pose)

            prev_node = node_list[i - 1]
            next_node = node_list[i + 1]

            dx1, dy1 = node.x - prev_node.x, node.y - prev_node.y
            dx2, dy2 = next_node.x - node.x, next_node.y - node.y

            # If direction changes, add to simplified path
            if dx1 * dy2 != dy1 * dx2:
                simplified_path_msg.poses.append(pose)

        # Ensure last pose is added to simplified path
        last_pose = PoseStamped()
        last_pose.header.frame_id = "map"
        last_pose.header.stamp = self.get_clock().now().to_msg()
        last_pose.pose.position.x = node_list[-1].x * self.config_space.info.resolution + self.config_space.info.origin.position.x
        last_pose.pose.position.y = node_list[-1].y * self.config_space.info.resolution + self.config_space.info.origin.position.y
        full_path_msg.poses.append(last_pose)
        simplified_path_msg.poses.append(last_pose)

        return simplified_path_msg, full_path_msg

    def a_star(self, node_start, node_goal, goal_threash):
        
        # self.get_logger().warn("Enters A*")
        open_dict = {}
        closed_dict = {}
        open_dict[(node_start.x, node_start.y)] = node_start
        while open_dict:
            # self.get_logger().warn(f"Length open dict: {len(open_dict)}")
            #node_current = open_dict[min(open_dict.keys(), key=lambda k: open_dict[k].f)] # Gets the node with the lowest f score
            node_current_key = min(open_dict.keys(), key=lambda k: open_dict[k].f)
            node_current = open_dict.pop(node_current_key)
            closed_dict[node_current.x, node_current.y] = node_current

            if goal_threash == None: # Exploration
                if node_current.x == node_goal.x and node_current.y == node_goal.y:
                    return self.construct_path(node_current)

            elif node_current.h < goal_threash: # Collection
                return self.construct_path(node_current)

            for node_child in node_current.get_children(node_goal, self.config_space, self.cost_ratio):
                child_key = (node_child.x, node_child.y)
                if child_key in closed_dict:
                    continue
                if child_key in open_dict:
                    node_open = open_dict[child_key] # Cell with the same index in open dict
                    if node_child.g < node_open.g: # Shorter path found
                        node_open.g = node_child.g
                        node_open.h = node_child.h 
                        node_open.f = node_child.f
                        node_open.parent = node_child.parent
                else:
                    # self.get_logger().warn("Add nodes to open dict")
                    open_dict[child_key] = node_child
        
        self.get_logger().warn("No valid path found (open_dict empty)")
        return None, None
    
    def world_to_grid(self, x, y):
        '''Converts world coordinates in [m] to grid indices.'''
        i_x = int((x - self.config_space.info.origin.position.x) / self.config_space.info.resolution)    
        i_y = int((y - self.config_space.info.origin.position.y) / self.config_space.info.resolution)
        return i_x, i_y
                    
# def main():
#     rclpy.init()
#     node = planner_A_star()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
