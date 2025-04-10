import rclpy
import rclpy.clock
import rclpy.duration
import rclpy.time
from rclpy.executors import MultiThreadedExecutor
import sensor_msgs_py
import numpy as np
import csv
import math
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from math import fabs
from rclpy.node import Node
import sensor_msgs_py.point_cloud2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from visualization_msgs.msg import MarkerArray
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from scipy.ndimage import binary_dilation, binary_fill_holes, convolve
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os


class OccupancyGridNode(Node):
    def __init__(self): 

        # Initializes
        super().__init__('update_occupancy_grid')
        self.publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.config_space_pub = self.create_publisher(OccupancyGrid, '/config_space', 10)
        self.lidar_subscription = self.create_subscription(LaserScan,'/scan',self.listener_callback,10)
        self.vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.current_object_goal_sub = self.create_subscription(PointStamped, '/goal_point', self.current_object_goal, 
                                                                rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.pop_obj_map_sub = self.create_subscription(Bool, '/object_rm', self.rm_object_goal,
                                                        rclpy.qos.QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))

        relative_path_to_file = os.path.join("/home/group3-robot/robp_group3/robp_ws/src/behavior_tree", "map_1.tsv")
        self.filename = os.path.realpath(relative_path_to_file) #introduce name of the text file
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) 
        self.proj = LaserProjection()
        
        # Gets workspace variables from tsv file
        vertices, min_x, max_x, min_y, max_y = self.read_workspace()
        
        # Grid parameters
        self.resolution = 0.04 # Grid cell size (m)
        self.width = math.ceil((max_x - min_x)/self.resolution)+2
        self.height = math.ceil((max_y - min_y)/self.resolution)+2      
        self.origin_x = min_x- self.resolution  
        self.origin_y = min_y- self.resolution
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)  # Occupancy grid
        self.config_space = self.grid # Config space
        self.goal_object_mask_inflated = np.zeros_like(self.grid)
        self.grid.fill(-1) # Sets all cells to unknown
        self.geofence_mask = None # Geofence mask to check if lidar points are inside the workspace
        self.geofence(vertices) # Sets a boundry for the workspace

        # Inflation parameter
        self.robot_radius = 0.3
        self.goal_x, self.goal_y = None, None

        self.angular_vel = 0.0

        # Obstacle tracking dictionary
        self.lidar_obstacles = {}  # {(x, y): timestamp}

        self.add_objects()
        self.publish_current_grid()

    def read_workspace(self):
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        tsv_file_path = '/home/group3-robot/robp_group3/robp_ws/src/mapping/mapping/workspace_3.tsv'
        vertices = [] # Stores verticies as (x, y) tuples
        with open(tsv_file_path, newline='') as tsvfile:
            reader = csv.reader(tsvfile, delimiter='\t') # List of lists  
            next(reader) # SKips the header
            for row in reader: # Each row is a list
                x, y = float(row[0])/100, float(row[1])/100 # Coordiantes in file (m)
                vertices.append((x, y))
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)
        return vertices, min_x, max_x, min_y, max_y

    def geofence(self, vertices):

        # Create an empty mask
        mask = np.zeros((self.height, self.width), dtype=bool)
        self.geofence_mask = np.zeros((self.height, self.width), dtype=bool)
        
        # Rasterize the geofence boundary
        for i in range(len(vertices)):
            x0, y0 = vertices[i]
            x1, y1 = vertices[(i + 1) % len(vertices)]
            start = self.world_to_grid(x0, y0)
            end = self.world_to_grid(x1, y1)
            cells = self.raytrace(start, end)
            for cell in cells:
                i_x, i_y = cell
                mask[i_y, i_x] = True
        
        # Fill inside the geofence
        filled_mask = binary_fill_holes(mask)

        # Mask with both border and outside of border
        #self.geofence_mask[mask] = True
        self.geofence_mask[~filled_mask] = True
        # Marks the occupancy grid 
        self.grid[self.geofence_mask] = 100 
    
    def add_objects(self):
        data = []
        with open(self.filename, 'r') as f:
            for line in f:
                line = line.strip() # remove leading and trailing whitespaces
                words = line.split('\t')
                data.append(words)

        objs_list = [[sublist[0]] + [0.01*float(x) for x in sublist[1:]] for sublist in data if sublist[0] != 'B']
        box_list = [[sublist[0]] + [0.01*float(x) for x in sublist[1:]] for sublist in data if sublist[0] == 'B']

        full_list = objs_list + box_list  # Combine both for a single path

        for obj in full_list:
            label, x, y = obj[0], obj[1], obj[2]
            i_x, i_y = self.world_to_grid(x, y)
            # Mark the main cell as an object
            self.grid[i_y][i_x] = 100

            # Check if msg.frame_id is 'B' before marking adjacent cells
            if label == 'B':
                inflation_radius = 2 

                for dy in range(-inflation_radius, inflation_radius + 1):
                    for dx in range(-inflation_radius, inflation_radius + 1):
                        adj_y = i_y + dy
                        adj_x = i_x + dx

                        # Ensure we don't go out of bounds
                        if 0 <= adj_y < self.height and 0 <= adj_x < self.width:
                            self.grid[adj_y][adj_x] = 100  # Mark as occupied
    
    
    def world_to_grid(self, x, y):
        '''Converts world coordinates in [m] to grid indices.'''
        i_x = int((x - self.origin_x) / self.resolution)    
        i_y = int((y - self.origin_y) / self.resolution)
        return i_x, i_y
    
    def raytrace(self, start, end):
        (x0, y0) = start
        (x1, y1) = end
        (dx, dy) = (fabs(x1 - x0), fabs(y1 - y0))
        sx = 1 if x0 < x1 else -1 # Different types of slopes described by sx and sy
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        traversed = []
        while x0 != x1 or y0 != y1:
            if 0 <= x0 < self.width and 0 <= y0 < self.height:
                traversed.append((x0, y0))
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy        
        return traversed # returns a lsit of cell indexes from start to one cell before end


    def inflate_map(self):
        binary_grid = np.zeros_like(self.grid)
        border_grid = np.zeros_like(self.grid)

        # Threshold for obstacles (usually >50 is considered occupied)
        binary_grid[(self.grid > 50) & (~self.geofence_mask)] = 1
        border_grid[self.geofence_mask] = 1
        
        # Define different kernel sizes
        large_radius = int(np.ceil(self.robot_radius / self.resolution))  # Larger for workspace
        small_radius = large_radius // 3  # Smaller for borders

        # Create circular kernels
        y_large, x_large = np.ogrid[-large_radius:large_radius+1, -large_radius:large_radius+1]
        kernel_large = x_large**2 + y_large**2 <= large_radius**2

        y_small, x_small = np.ogrid[-small_radius:small_radius+1, -small_radius:small_radius+1]
        kernel_small = x_small**2 + y_small**2 <= small_radius**2

        # Apply dilation to the entire grid
        inflated_grid = binary_dilation(binary_grid, kernel_large)
        inflated_border = binary_dilation(border_grid, kernel_small)

        # Combine the two results
        self.config_space = (((inflated_grid & ~(self.goal_object_mask_inflated))| inflated_border) * 99).astype(np.int8)


        # Create an OccupancyGrid message
        config_grid_msg = OccupancyGrid()
        config_grid_msg.header = Header()
        config_grid_msg.header.stamp = self.get_clock().now().to_msg()
        config_grid_msg.header.frame_id = "map"
        config_grid_msg.info.resolution = self.resolution
        config_grid_msg.info.width = self.width
        config_grid_msg.info.height = self.height
        config_grid_msg.info.origin.position.x = self.origin_x
        config_grid_msg.info.origin.position.y = self.origin_y
        config_grid_msg.info.origin.orientation.w = 1.0
        config_grid_msg.data = self.config_space.flatten().tolist()

        self.config_space_pub.publish(config_grid_msg)

    def publish_current_grid(self):
        """Publish the occupancy grid using the current grid data."""
        occupancy_grid_msg = OccupancyGrid() 
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'map'
        occupancy_grid_msg.info.resolution = self.resolution
        occupancy_grid_msg.info.width = self.width
        occupancy_grid_msg.info.height = self.height
        occupancy_grid_msg.info.origin.position.x = self.origin_x
        occupancy_grid_msg.info.origin.position.y = self.origin_y
        occupancy_grid_msg.info.origin.orientation.w = 1.0 # No rotation applied
        occupancy_grid_msg.data = self.grid.flatten().tolist()
        self.publisher.publish(occupancy_grid_msg)
        # self.get_logger().info("Published occupancy grid")
    
    def vel_callback(self, msg):
        self.angular_vel = msg.angular.z

    def listener_callback(self, msg):
        # Looks up transform from lidar link to map
        to_frame_rel = 'map'
        time = rclpy.time.Time().from_msg(msg.header.stamp)

        if abs(self.angular_vel) < 0.2:
            lidar_from_frame_rel = msg.header.frame_id # Lidar link
            lidar_tf_future = self.tf_buffer.wait_for_transform_async(to_frame_rel, lidar_from_frame_rel, time)
            lidar_tf_future.add_done_callback(lambda future: self.lidar_transform_callback(future, msg))


    def lidar_transform_callback(self, future, msg):
        try:    
            t_lidar = future.result()  # Get the transform when ready
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {msg.header.frame_id} to map: {ex}')
            return

        # Gets the robots position in the map frame
        robot_x = t_lidar.transform.translation.x
        robot_y = t_lidar.transform.translation.y 
        robot_index = self.world_to_grid(robot_x, robot_y)

        # Clone the LaserScan message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = max(msg.range_min, 0.2)
        filtered_scan.range_max = min(msg.range_max, 2.0)  # Limit max range to 2m
        filtered_scan.intensities = msg.intensities

        # Filter out points beyond 5 meters
        filtered_scan.ranges = [r if r <= 2.0 and r >= 0.2 else float('inf') for r in msg.ranges]
                    

        # Project LaserScan to PointCloud2
        cloud = self.proj.projectLaser(filtered_scan)
        cloud_map = do_transform_cloud(cloud, t_lidar)
        
        now = self.get_clock().now()
        current_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9 
        

        for point in sensor_msgs_py.point_cloud2.read_points(cloud_map, field_names=("x", "y"), skip_nans=True):
            x, y = self.world_to_grid(point[0], point[1])
            if 0 <= x < self.width and 0 <= y < self.height:
                if not self.geofence_mask[y, x]:  # Only add points if its not part of the geofence mask
                    self.lidar_obstacles[(x, y)] = current_time

        to_remove = []
        for (x, y), timestamp in self.lidar_obstacles.items():
            elapsed_time = (current_time - timestamp) # Elapsed time will be 0.1 seconds since the lidar updates at 10hz 
            new_value = max(0, int(99 - 10*elapsed_time))

            if new_value == 0:
                to_remove.append((x, y))
                self.grid[y, x] = -1
            else:
                self.grid[y, x] = new_value  # Update the grid with decayed value
        
        for key in to_remove:
            del self.lidar_obstacles[key]

        self.publish_current_grid()
        self.rm_loners()
        self.inflate_map()

    def rm_loners(self):
        """Removes lidar occupied cells (0<=(cell value)<=99) without any neighbors"""
        mask = (self.grid > 0) & (self.grid < 100)
        kernel = np.array([[0, 1, 0],
                           [1, 0, 1],
                           [0, 1, 0]])
        nr_neighbors = convolve(mask, kernel)
        new_grid = self.grid.copy()
        new_grid[mask & (nr_neighbors == 0)] = -1
        self.grid = new_grid

    
    def current_object_goal(self, msg:PointStamped):
        mask = np.zeros_like(self.grid)
        x_w, y_w = msg.point.x, msg.point.y
        self.goal_x, self.goal_y = self.world_to_grid(x_w, y_w)
        mask[self.goal_y, self.goal_x] = 1
        r = int(np.ceil(self.robot_radius / self.resolution)) 

        y_inflate, x_inflate = np.ogrid[-r:r+1, -r:r+1]
        kernel = x_inflate**2 + y_inflate**2 <= r**2

        mask = binary_dilation(mask, kernel)

        self.goal_object_mask_inflated =  mask

    def rm_object_goal(self, msg:Bool):
        if msg.data and self.goal_y is not None and self.goal_y is not None:
            self.grid[self.goal_y, self.goal_x] = -1



def main():
    rclpy.init()
    node = OccupancyGridNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    