import rclpy
<<<<<<< HEAD
import math
import numpy as np
import heapq
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from scipy.ndimage import binary_dilation
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from mapping.mapping.occupancy_grid import OccupancyGridNode

class Nodes:
        def __init__(self, x, y):
            self.x = x # Observe that these are occupancy grid indexes
=======
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
>>>>>>> origin/collection_bt
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
            
<<<<<<< HEAD
            # Check if in free space or unknown (0) rather than obstacle (1)
            return config_space[self.y][self.x] == 0

        def get_children(self, node_goal, config_space, map_info, cost_ratio):
            ok_children_list = []
=======
            # Check if in free space (0) rather than obstacle (1)
            return config_space[self.y][self.x] == 0

        def get_children(self, node_goal, config_space, map_info):
            ok_children_list = []
            cost_ratio = 5
>>>>>>> origin/collection_bt
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

<<<<<<< HEAD
class Planner_A_star(Node):
    def __init__(self):
        super().__init__('planner_A_star')
        self.simple_publisher = self.create_publisher(Path ,'/motion/path' , 10)
        self.full_publisher = self.create_publisher(Path ,'/full_path' , 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/occupancy_grid', self.map_callback, 10)
        self.goal_subscription = self.create_subscription(PointStamped, '/goal_point', self.goal_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) 
=======
class planner_A_star(Node):
    def __init__(self):
        super().__init__('planner_A_star')
        self.publisher = self.create_publisher(Path ,'/path' , 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_subscription = self.create_subscription(PointStamped, '/temp_goal', self.goal_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) 
>>>>>>> origin/collection_bt

        # Initizialize
        self.robot_radius = 0.20
        self.cost_ratio = 5
        self.config_space = None
<<<<<<< HEAD
        #self.map_info = None
        self.occupancy_grid_msg =None
        self.goal_msg = None
    
    def goal_callback(self, msg):
        self.goal_msg = msg
        
    def map_callback(self, msg): # Create the configurations space
        self.occupancy_grid_msg = msg        
    
    def inflate_map(self, occupancy_grid_msg):
        grid = np.array(occupancy_grid_msg.data).reshape(occupancy_grid_msg.info.height, occupancy_grid_msg.info.width)
        binary_grid = np.zeros_like(grid)
        binary_grid[grid > 50] = 1
=======
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
        
>>>>>>> origin/collection_bt
        # Calculate kernel size based on robot radius and map resolution
        kernel_radius = int(np.ceil(self.robot_radius / self.map_info.resolution))
        
        # Create circular kernel for dilation
        y, x = np.ogrid[-kernel_radius:kernel_radius+1, -kernel_radius:kernel_radius+1]
        kernel = x**2 + y**2 <= kernel_radius**2
        
        # Dilate obstacles to create configuration space
        self.config_space = binary_dilation(binary_grid, kernel).astype(np.int8)
<<<<<<< HEAD
        self.get_logger().info(f'Configuration space created with robot radius: {self.robot_radius}m')
    
    def world_to_grid(self, x, y):
        '''Converts world coordinates in [m] to grid indices.'''
        i_x = int((x - self.origin_x) / self.resolution)    
        i_y = int((y - self.origin_y) / self.resolution)
        return i_x, i_y
    
    def path_plan(self): # Called from the behavior   
        # Get current position
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))  # I want the latest possible transform      
            start_x = t.transform.translation.x
            start_y = t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Lookup failed: {str(e)}")
            return
        if self.map_info is None:
            self.get_logger().warn('Occuapncy grid not recived for path planning')
            return
        if self.goal_msg is None:
            self.get_logger().warn('Goal point not recived for path planning')
            return
        
        # Inflate map
        self.inflate_map(self.occupancy_grid_msg, only_obstacle=False)

        # Convert world to grid (using inherited function)
        i_start_x, i_start_y = self.world_to_grid(start_x, start_y)
        i_goal_x, i_goal_y = self.world_to_grid(self.goal_msg.point.x, self.goal_msg.point.y)

        node_goal = Nodes(i_goal_x, i_goal_y)
        node_start = Nodes(i_start_x, i_start_y)
        node_start.g = 0
        node_start.h = ((i_goal_x - i_start_x)**2 + (i_goal_y - i_start_y)**2)**0.5
        node_start.f = self.cost_ratio*node_start.h + node_start.g
        
        # Publish path
        simplified_path, full_path = self.a_star(node_start, node_goal)
        self.simple_publisher.publish(simplified_path)
        self.full_publisher.publish(full_path)
        
    def construct_path(self, node_current):
        node_list = [node_current]
        while node_current.parent != None:
            node_list.append(node_current.parent)
            node_current = node_current.parent

        node_list.reverse()
        full_path = Path()
        simplified_path = [node_list[0]]  # Start with the first node

        for i, node in enumerate(node_list):
            # Construct full_path
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = node.x * self.map_info.resolution + self.map_info.origin.position.x
            pose.pose.position.y = node.y * self.map_info.resolution + self.map_info.origin.position.y
            full_path.poses.append(pose)

            # Skip first and last nodes for collinearity check
            if 0 < i < len(node_list) - 1:
                prev = simplified_path[-1]
                next = node_list[i + 1]
                dx1, dy1 = node.x - prev.x, node.y - prev.y
                dx2, dy2 = next.x - node.x, next.y - node.y
                
                # If directions change, keep this node
                if dx1 * dy2 != dy1 * dx2:
                    simplified_path.append(node)

        simplified_path.append(node_list[-1])  # Always keep the last node
        return simplified_path, full_path
=======
        
        self.get_logger().info(f'Configuration space created with robot radius: {self.robot_radius}m')

    def construct_path(self, node_current):
        path = [node_current]
        while node_current.parent != None:
            path.append(node_current.parent)
            node_current = node_current.parent
        return path[::-1]
>>>>>>> origin/collection_bt

    def a_star(self, node_start, node_goal):
        open_dict = {}
        closed_dict = {}
        open_dict[(node_start.x, node_start.y)] = node_start

        while open_dict:
            node_current = open_dict[min(open_dict.keys(), key=lambda k: open_dict[k].f)] # Gets the node with the lowest f score
            closed_dict[node_current.x, node_current.y] = node_current
<<<<<<< HEAD
            goal_distance = node_current.h * self.map_info.resolution
            if goal_distance < 0.1: 
                return self.construct_path(node_current)

            for node_child in node_current.get_children(node_goal, self.config_space, self.map_info, self.cost_ratio):
=======
            if node_current.h < 0.1:
                path = self.construct_path(node_current)
                break

            for node_child in node_current.get_children(node_goal, self.config_space, self.map_info):
>>>>>>> origin/collection_bt
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
<<<<<<< HEAD
        
        self.get_logger().warn("No valid path found (open_dict empty)")
        return []
                    
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
=======
                    
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
>>>>>>> origin/collection_bt
