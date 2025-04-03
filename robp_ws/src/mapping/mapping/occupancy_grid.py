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
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from scipy.ndimage import binary_dilation, binary_fill_holes, convolve

# free space from lidar: not marked
# free space from camera: 0 
# Occupied by lidar: 99
# Occupied by camera: 100

class OccupancyGridNode(Node):
    def __init__(self): 

        # Initializes
        super().__init__('update_occupancy_grid')
        self.publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.config_space_pub = self.create_publisher(OccupancyGrid, '/config_space', 10)
        self.lidar_subscription = self.create_subscription(LaserScan,'/scan',self.listener_callback,10)
        self.vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.object_sub = self.create_subscription(MarkerArray, '/object_positions', self.obj_callback, 10)
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
        self.grid.fill(-1) # Sets all cells to unknown
        self.geofence_mask = None # Geofence mask to check if lidar points are inside the workspace
        self.geofence(vertices) # Sets a boundry for the workspace

        # Inflation parameter
        self.robot_radius = 0.2
        
        # Camera paramters
        self.camera_FOV = 90 # np.pi/2 # Mapping should run all the time but how?
        self.camera_min_range = 0.2 # True value: 0.2
        self.camera_max_range = 0.82 # True value: 3.0

        self.angular_vel = 0.0

        # Obstacle tracking dictionary
        self.lidar_obstacles = {}  # {(x, y): timestamp}

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
        self.geofence_mask[mask] = True
        self.geofence_mask[~filled_mask] = True

        # Marks the occupancy grid 
        self.grid[self.geofence_mask] = 100 
    
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
        # self.rm_loners()   # We might need this for path planning. The config space updates quicker than the map gets published and doesn't take loners into account. 
        binary_grid = np.zeros_like(self.grid)
        
        # Threshold for obstacles (usually >50 is considered occupied)
        binary_grid[self.grid > 50] = 1
        # Robot can drive through both known and unknown space
        
        # Calculate kernel size based on robot radius and map resolution
        kernel_radius = int(np.ceil(self.robot_radius / self.resolution))
        
        # Create circular kernel for dilation
        y, x = np.ogrid[-kernel_radius:kernel_radius+1, -kernel_radius:kernel_radius+1]
        kernel = x**2 + y**2 <= kernel_radius**2
        
        # Dilate obstacles to create configuration space
        self.config_space = binary_dilation(binary_grid, kernel).astype(np.int8)*100

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

        if abs(self.angular_vel) < 0.1:
            lidar_from_frame_rel = msg.header.frame_id # Lidar link
            lidar_tf_future = self.tf_buffer.wait_for_transform_async(to_frame_rel, lidar_from_frame_rel, time)
            lidar_tf_future.add_done_callback(lambda future: self.lidar_transform_callback(future, msg))

        # Looks up transfrom from camera_depth_optical_frame to map (uses the same time)
        camera_from_frame = 'camera_depth_optical_frame'
        camera_tf_future = self.tf_buffer.wait_for_transform_async(to_frame_rel, camera_from_frame, time)
        camera_tf_future.add_done_callback(lambda future: self.camera_transform_callback(future, msg))

        self.inflate_map()

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
            new_value = max(0, int(99 - elapsed_time))

            if new_value == 0:
                to_remove.append((x, y))
                self.grid[y, x] = -1
            else:
                self.grid[y, x] = new_value  # Update the grid with decayed value
        
        for key in to_remove:
            del self.lidar_obstacles[key]


        # self.publish_current_grid() 
    
    def camera_transform_callback(self, future, msg):
        try:
            t_cam = future.result()
        except TransformException as ex:
            self.get_logger().info(f'Could not transform camera_depth_optical_frame to map: {ex}')
            return

        # Get the camera's position in the map frame
        cam_x = t_cam.transform.translation.x
        cam_y = t_cam.transform.translation.y
        
        # Convert quaternion to yaw angle (assuming the camera is nearly horizontal)
        q = t_cam.transform.rotation
        # siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        # cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        # yaw = math.atan2(siny_cosp, cosy_cosp)
        
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = euler[2] + np.pi/2

        # Sweep rays over the camera FOV (assumed centered around the optical axis)
        num_rays = 30  # You can adjust the number of rays for resolution
        start_angle = - np.deg2rad(self.camera_FOV) / 2
        end_angle = np.deg2rad(self.camera_FOV) / 2
        
        for i in range(num_rays):
            angle = start_angle + i * (end_angle - start_angle) / (num_rays - 1)
            ray_angle = yaw + angle
            # Compute endpoint of the ray at the camera's max range
            end_x = cam_x + self.camera_max_range * np.cos(ray_angle) 
            end_y = cam_y + self.camera_max_range * np.sin(ray_angle)
            start_x = cam_x + self.camera_min_range * np.cos(ray_angle)
            start_y = cam_y + self.camera_min_range * np.sin(ray_angle)
            
            start_i = self.world_to_grid(start_x, start_y)
            end_i = self.world_to_grid(end_x, end_y)
            cells = self.raytrace(start_i, end_i)
            
            for cell in cells:
                i_x, i_y = cell
                if 0 <= i_x < self.width and 0 <= i_y < self.height:
                    # Mark as known by camera (free) if not already a fence/occupied by lidar
                    if self.grid[i_y, i_x] < 50:
                        self.grid[i_y, i_x] = 0

    #TODO: Might want to remove old detections first
    def obj_callback(self, msg:MarkerArray):
        for marker in msg.markers:
            i_x, i_y = self.world_to_grid(marker.pose.position.x*0.01, marker.pose.position.y*0.01)

            # Ensure x, y are within the grid bounds
            if 0 <= i_x < self.width and 0 <= i_y < self.height:
                # Mark the main cell as an object
                self.grid[i_y][i_x] = 100

                # Check if msg.frame_id is 'B' before marking adjacent cells
                if marker.header.frame_id == 'B':
                    # Define four possible directions: (dy, dx)
                    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

                    for dy, dx in directions:
                        for step in range(1, 3):  # Expand 1 step and 2 steps
                            adj_y, adj_x = i_y + step * dy, i_x + step * dx

                            # Ensure we don't go out of bounds
                            if 0 <= adj_y < self.height and 0 <= adj_x < self.width:
                                self.grid[adj_y][adj_x] = 100  # Mark adjacent cell as an object
                  
        self.inflate_map()

    def rm_loners(self):
        """Removes lidar occupied cells (50<=(cell value)<=99) without any neighbors"""
        mask = (self.grid >= 50) & (self.grid <= 99)
        kernel = np.array([[0, 1, 0],
                           [1, 0, 1],
                           [0, 1, 0]])
        nr_neighbors = convolve(mask, kernel)
        new_grid = self.grid.copy()
        new_grid[mask & (nr_neighbors == 0)] = -1
        self.grid = new_grid

    # def rm_loners(self):
    #     nr_occupied_by_lidar = 0
    #     nr_lone = 0
    #     for y in range(self.height):
    #         for x in range(self.width):
    #             if self.grid[y, x] >= 50 and self.grid[y, x] <= 99:
    #                 nr_occupied_by_lidar += 1
    #                 nr_neighbors = 0
    #                 if x-1 >= 0:
    #                     if self.grid[y,x-1] >= 50 and self.grid[y,x-1] >= 99:
    #                         nr_neighbors += 1
    #                 if x+1 <= self.width-1:
    #                     if self.grid[y,x+1] >= 50 and self.grid[y,x+1] <= 99:
    #                         nr_neighbors += 1
    #                 if y-1 >= 0:
    #                     if self.grid[y-1,x] >= 50 and self.grid[y-1,x] <= 99:
    #                         nr_neighbors += 1
    #                 if y+1 <= self.height-1:
    #                     if self.grid[y+1,x] >= 50 and self.grid[y+1,x] <= 99:
    #                         nr_neighbors += 1
    #                 #self.get_logger().info(f'nr neighbors {nr_neighbors}')
    #                 if nr_neighbors == 0:
    #                     self.grid[y,x] = -1
    #                     nr_lone += 1
    #     self.get_logger().info(f'jjjjjjjjjjjjjjjjjjj{nr_occupied_by_lidar}, {nr_lone}')


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
    