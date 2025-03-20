import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.ndimage import binary_dilation
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Nodes:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.g = None
            self.h = None
            self.f = None
            self.parent = None

        def __lt__(self, other):
            return self.f < other.f

        def is_ok(self, config_space, map_info):
            # Check if within map bounds
            if (self.x < 0 or self.x >= map_info.width or 
                self.y < 0 or self.y >= map_info.height):
                return False
            
            # Check if in free space (0) rather than obstacle (1)
            return config_space[self.y][self.x] == 0

        def get_children(self, node_goal, config_space, map_info):
            ok_children_list = []
            cost_ratio = 5
            step_size = 1
            directions = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]

            children_list = [Nodes(self.x + dx * step_size, self.y + dy * step_size) for dx, dy in directions]
            for node_child in children_list:
                if node_child.is_ok(config_space, map_info):
                    node_child.g = self.g + ((self.x - node_child.x)**2 + (self.y - node_child.y)**2)**0.5 
                    node_child.h = ((node_goal.x - node_child.x)**2 + (node_goal.y - node_child.y)**2)**0.5
                    node_child.f = cost_ratio*node_child.h + node_child.g
                    node_child.parent = self
                    ok_children_list.append(node_child)
            return ok_children_list

class planner_A_star(Node):
    def __init__(self):
        super().__init__('planner_A_star')
        self.publisher = self.create_publisher(Path ,'/path' , 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_subscription = self.create_subscription(PointStamped, '/temp_goal', self.goal_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) 

        # Initizialize
        self.robot_radius = 0.20
        self.cost_ratio = 5
        self.config_space = None
        self.map_info = None

    def world_to_grid(self, x, y):
        '''Converts world coordinates in [m] to grid indices.'''
        i_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)    
        i_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return i_x, i_y
    
    def goal_callback(self, msg): 
        if self.config_space is None:
            self.get_logger().warn('Received goal but map or configuration space not available yet')
            return
        
        # Get current position
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))  # Timeout for lookup)        
            start_x = t.transform.translation.x
            start_y = t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Lookup failed: {str(e)}")

        # Convert world to grid
        i_start_x, i_start_y = self.world_to_grid(start_x, start_y)
        i_goal_x, i_goal_y = self.world_to_grid(msg.point.x, msg.point.y)

        node_start = Nodes(i_start_x, i_start_y)
        node_start.g = 0
        node_start.h = ((i_goal_x - i_start_x)**2 + (i_goal_y - i_start_y)**2)**0.5
        node_start.f = self.cost_ratio*node_start.h + node_start.g
        node_goal = Nodes(i_goal_x, i_goal_y) 
        self.a_star(node_start, node_goal)

    def map_callback(self, occupancy_grid_msg): # Create the configurations space
        # The problem with doing path palnning when there is a new map topic is 
        # Convert occupancy grid to numpy array
        self.map_info = occupancy_grid_msg.info # Gets info from the occupancy grid

        # Create binary occupancy grid (1 for obstacles, 0 for free space)
        grid = np.array(occupancy_grid_msg.data).reshape(self.map_info.height, self.map_info.width)
        binary_grid = np.zeros_like(grid)
        
        # Threshold for obstacles (usually >50 is considered occupied)
        binary_grid[grid > 50] = 1
        binary_grid[grid < 0] = 1  # Treat unknown space as obstacles too (Hmmmm???)
        
        # Calculate kernel size based on robot radius and map resolution
        kernel_radius = int(np.ceil(self.robot_radius / self.map_info.resolution))
        
        # Create circular kernel for dilation
        y, x = np.ogrid[-kernel_radius:kernel_radius+1, -kernel_radius:kernel_radius+1]
        kernel = x**2 + y**2 <= kernel_radius**2
        
        # Dilate obstacles to create configuration space
        self.config_space = binary_dilation(binary_grid, kernel).astype(np.int8)
        
        self.get_logger().info(f'Configuration space created with robot radius: {self.robot_radius}m')

    def construct_path(self, node_current):
        path = [node_current]
        while node_current.parent != None:
            path.append(node_current.parent)
            node_current = node_current.parent
        return path[::-1]

    def a_star(self, node_start, node_goal):
        open_dict = {}
        closed_dict = {}
        open_dict[(node_start.x, node_start.y)] = node_start

        while open_dict:
            node_current = open_dict[min(open_dict.keys(), key=lambda k: open_dict[k].f)] # Gets the node with the lowest f score
            closed_dict[node_current.x, node_current.y] = node_current
            if node_current.h < 0.1:
                path = self.construct_path(node_current)
                break

            for node_child in node_current.get_children(node_goal, self.config_space, self.map_info):
                child_key = (node_child.x, node_child.y)
                if child_key in closed_dict:
                    continue
                if child_key in open_dict:
                    node_open = open_dict[child_key]
                    if node_child.g < node_open.g: # Shorter path found
                        node_open.g = node_child.g
                        node_open.h = node_open.h 
                        node_open.f = node_child.f
                        node_open.parent = node_child.parent
                else:
                    open_dict[child_key] = node_child
                    
def main():
    rclpy.init()
    node = planner_A_star()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()