import rclpy
import rclpy.duration
import rclpy.time
import sensor_msgs_py
import numpy as np

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


class OccupancyGridNode(Node):
    def __init__(self):

        # Initializes
        super().__init__('occupancy_grid')
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10) 
        self.subscription = self.create_subscription(LaserScan,'/scan',self.listener_callback,10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) 
        self.proj = LaserProjection() 

        # Grid parameters
        self.resolution = 0.05  # Grid cell size (meters)
        self.width = 40  # Grid width (cells)
        self.height = 40  # Grid height (cells)
        self.origin_x = 0  # Bottom-left corner in the map frame 
        self.origin_y = 0
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)  # Occupancy grid
        self.grid.fill(-1) # Sets all cells to unknown
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices."""
        i_x = int((x - self.origin_x) / self.resolution)
        i_y = int((y - self.origin_y) / self.resolution)
        return i_x, i_y
    
    def update_grid(self, start_x, start_y, end_x, end_y):
        """Update the occupancy grid using Bresenham's line algorithm."""
        x0, y0 = self.world_to_grid(start_x, start_y)
        x1, y1 = self.world_to_grid(end_x, end_y)

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        # Mark free cells along the ray
        while x0 != x1 or y0 != y1:
            if 0 <= x0 < self.width and 0 <= y0 < self.height:
                self.grid[y0, x0] = self.free_threshold
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        # Mark the end point as occupied
        if 0 <= x1 < self.width and 0 <= y1 < self.height:
            self.grid[y1, x1] = self.occupied_threshold

    def publish_grid(self, msg):
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = rclpy.time.Time().from_msg(msg.header.stamp)
        occupancy_grid_msg.header.frame_id = 'map'
        occupancy_grid_msg.info.resolution = self.resolution
        occupancy_grid_msg.info.width = self.width
        occupancy_grid_msg.info.height = self.height
        occupancy_grid_msg.info.origin.position.x = self.origin_x
        occupancy_grid_msg.info.origin.position.y = self.origin_y
        occupancy_grid_msg.info.origin.orientation.w = 1.0
        occupancy_grid_msg.data = self.grid.flatten().tolist()
        self.publisher.publish(occupancy_grid_msg)

    
    def listener_callback(self, msg):
        
        # Looks up transform from lidar link to map
        to_frame_rel = 'map'
        from_frame_rel = msg.header.frame_id # Lidar link
        time = rclpy.time.Time().from_msg(msg.header.stamp)
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # Gets the robots position in the map frame
        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y 

        # Project LaserScan to PointCloud2
        cloud = self.proj.projectLaser(msg)
        cloud_map = do_transform_cloud(cloud, t)

        for point in sensor_msgs_py.point_cloud2.read_points(cloud_map, field_names=("x", "y"), skip_nans=True):
            x, y = point[0], point[1]
            self.update_grid(robot_x, robot_y, x, y)
        
        # Publishes a new grid
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
