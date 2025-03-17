import rclpy
import rclpy.duration
import rclpy.time
import sensor_msgs_py
import numpy as np
import csv
import math

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
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from scipy.ndimage import binary_dilation

class OccupancyGridNode(Node):
    def __init__(self):

        # Initializes
        super().__init__('occupancy_grid')
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10) 
        self.lidar_subscription = self.create_subscription(LaserScan,'/scan',self.listener_callback,10)
        self.camera_subscription = self.create_subscription()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) 
        self.proj = LaserProjection()
        
        # Gets workspace variables from tsv file
        vertices, min_x, max_x, min_y, max_y = self.read_workspace()
        
        # Grid parameters
        self.resolution = 0.02 # Grid cell size (m)
        self.width = int((max_x - min_x)/self.resolution + 2)
        self.height = int((max_y - min_y)/self.resolution + 2)      
        self.origin_x = min_x - self.resolution 
        self.origin_y = min_y - self.resolution
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)  # Occupancy grid
        self.grid.fill(-1) # Sets all cells to unknown
        self.geofence(vertices) # Sets a boundry for the workspace
        # known with the lidar: 0
        # known with the camera: 1
        # Occupied by lidar: 100
        # Occupied by camera: 99
        
        # Camera paramters
        self.camera_FOV = np.pi/2 # Mapping should run all the time but how?
        self.camera_min_range = 0.3 # True value: 0.2
        self.camera_max_range = 0.8 # True value: 3.0

    def read_workspace(self):
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        tsv_file_path = '/home/group3-robot/robp_group3/robp_ws/src/mapping/mapping/workspace_2.tsv'
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
        num_vertices = len(vertices)
        for i in range(num_vertices):
            x0, y0 = vertices[i]
            x1, y1 = vertices[ (i+1) % num_vertices] 
            start = self.world_to_grid(x0, y0)
            end = self.world_to_grid(x1, y1)
            cells = self.raytrace(start, end)
            for cell in cells:
                (i_x, i_y) = cell
                self.grid[i_y, i_x] = 100

        # Makes the grid thicker to avoid ray going through the fences on diagonals   
        #dilated_grid = binary_dilation(self.grid == 100, iterations=1).astype(int) * 100
        #self.grid = np.maximum(self.grid, dilated_grid)
    
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

    def publish_grid(self, msg):
        occupancy_grid_msg = OccupancyGrid() # Creates new occupancy grid msg
        occupancy_grid_msg.header = Header() # Creates header for the occupancy grid msg
        occupancy_grid_msg.header.stamp = msg.header.stamp 
        occupancy_grid_msg.header.frame_id = 'map'
        occupancy_grid_msg.info.resolution = self.resolution
        occupancy_grid_msg.info.width = self.width
        occupancy_grid_msg.info.height = self.height
        occupancy_grid_msg.info.origin.position.x = self.origin_x
        occupancy_grid_msg.info.origin.position.y = self.origin_y
        occupancy_grid_msg.info.origin.orientation.w = 1.0 # No rotation applied
        occupancy_grid_msg.data = self.grid.flatten().tolist()
        self.publisher.publish(occupancy_grid_msg)
    
    def listener_callback(self, msg):
        # Looks up transform from lidar link to map
        to_frame_rel = 'map'
        time = rclpy.time.Time().from_msg(msg.header.stamp)
        
        lidar_from_frame_rel = msg.header.frame_id # Lidar link
        lidar_tf_future = self.tf_buffer.wait_for_transform_async(to_frame_rel, lidar_from_frame_rel, time)
        lidar_tf_future.add_done_callback(lambda future: self.lidar_transform_callback(future, msg))

        # DO THE FOLLOWING TWO THINGS:

        # Looks up transfrom from camera_depth_optical_frame to map (use the same variable time)
        camera_from_frame = 'camera_depth_optical_frame'
        camera_tf_future = self.tf_buffer.wait_for_transform_async(to_frame_rel, camera_from_frame, time)
        camera_tf_future.add_done_callback(lambda future: self.camera_transform_callback(future, msg))

        # Mark cells between the max and min range and FOV relative to the camera_depth_optical_frame as 1 in the occupancy grid

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

        # Project LaserScan to PointCloud2
        cloud = self.proj.projectLaser(msg)
        cloud_map = do_transform_cloud(cloud, t_lidar)

        for point in sensor_msgs_py.point_cloud2.read_points(cloud_map, field_names=("x", "y"), skip_nans=True):
            object_index = self.world_to_grid(point[0], point[1])
            cells = self.raytrace(robot_index, object_index)
            
            for cell in cells:
                i_x, i_y = cell
                if 0 <= i_x < self.width and 0 <= i_y < self.height:
                    if self.grid[i_y, i_x] != 100:
                        self.grid[i_y, i_x] = 0 # Mark as unoccupied
                    else:
                        break
            
            # Mark endpoint as occupied (if within bounds)
            if 0 <= object_index[0] < self.width and 0 <= object_index[1] < self.height:
                self.grid[object_index[1], object_index[0]] = 100  # occupied

        # Publishes a new grid
        self.publish_grid(msg)
    
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
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Sweep rays over the camera FOV (assumed centered around the optical axis)
        num_rays = 30  # You can adjust the number of rays for resolution
        start_angle = -self.camera_FOV / 2
        end_angle = self.camera_FOV / 2
        
        for i in range(num_rays):
            angle = start_angle + i * (end_angle - start_angle) / (num_rays - 1)
            ray_angle = yaw + angle
            # Compute endpoint of the ray at the camera's max range
            end_x = cam_x + self.camera_max_range * np.cos(ray_angle) 
            end_y = cam_y + self.camera_max_range * np.sin(ray_angle)
            start_x = cam_x + self.camera_min_range * np.cos(ray_angle)
            start_y = cam_x + self.camera_min_range * np.sin(ray_angle)
            
            start_i = self.world_to_grid(start_x, start_y)
            end_i = self.world_to_grid(end_x, end_y)
            cells = self.raytrace(start_i, end_i)
            
            for cell in cells:
                i_x, i_y = cell
                if 0 <= i_x < self.width and 0 <= i_y < self.height:
                    # Mark as known by camera (free) if not already a fence/occupied by lidar
                    if self.grid[i_y, i_x] != 100:
                        self.grid[i_y, i_x] = 1
        # Optionally, publish the updated grid after processing camera data.
        self.publish_grid(msg)
    
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
