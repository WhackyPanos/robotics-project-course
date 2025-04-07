import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped
from random import uniform
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from scipy.ndimage import binary_dilation

class RandomPoint(Node):
    def __init__(self):
        super().__init__('random_point')
        
        # Subscriptions
        self.space_subscription = self.create_subscription(OccupancyGrid, '/config_space', self.config_space_callback, 10)
        self.grid_subscription = self.create_subscription(OccupancyGrid, '/occupancy_grid', self.grid_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) 

        # Publisher
        self.publisher = self.create_publisher(PointStamped, '/goal_point', 10)
        
        # Map attributes
        self.config_space_data = None
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 1.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        #Robot's pose
        self.robot_x = self.robot_y = self.robot_yaw = None
        self.robot_radius = 0.35
    
    def config_space_callback(self, msg:OccupancyGrid):
        self.config_space_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    # def inflate_map(self):
    #     binary_grid = np.zeros_like(self.map_data)
        
    #     # Threshold for obstacles (usually >50 is considered occupied)
    #     binary_grid[self.map_data > 50] = 1
    #     # Robot can drive through both known and unknown space
        
    #     # Calculate kernel size based on robot radius and map resolution
    #     kernel_radius = int(np.ceil(self.robot_radius / self.map_resolution))
        
    #     # Create circular kernel for dilation
    #     y, x = np.ogrid[-kernel_radius:kernel_radius+1, -kernel_radius:kernel_radius+1]
    #     kernel = x**2 + y**2 <= kernel_radius**2
        
    #     # Dilate obstacles to create configuration space
    #     self.config_space_data = binary_dilation(binary_grid, kernel).astype(np.int8)


    def grid_callback(self, msg):
        """ Stores occupancy grid data (-1 unexplored, 0 explored, 99 object, 100 occupied). """
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    # def space_callback(self, msg):
    #     """ Stores configuration space (1 = occupied, 0 = free). """
    #     self.space_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def get_robot_pose(self):
        """ Retrieves the robot's current pose in the 'map' frame. """
        try:
            # Lookup transform from 'base_link' to 'map'
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())

            # Extract translation (position)
            position = transform.transform.translation
            x, y, z = position.x, position.y, position.z

            # Extract rotation (quaternion to Euler)
            orientation = transform.transform.rotation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            self.robot_x = x
            self.robot_y = y
            self.robot_yaw = yaw

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Transform lookup failed: {ex}")

    def is_valid_point(self, x, y):
        """ Checks if a point is free in config_space (0) and unexplored in occupancy_grid (-1). """
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)

        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            # Ensure point is in free space (config_space == 0) and not explored (occupancy_grid == -1)
            return self.config_space_data[grid_y, grid_x] == 0 and self.map_data[grid_y, grid_x] == -1
        return False

    def generate_new_point(self):
        """ Generates a random valid point in unexplored free space and publishes it. """
        if self.map_data is None or self.config_space_data is None:
            self.get_logger().warn("Map or config space data not available yet!")
            return False
        self.get_robot_pose()
        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None: 
            self.get_logger().warn("Robot pose not available yet!")
            return False
        
        # self.inflate_map()
        best_point = None
        best_score = -float("inf")  # We maximize this score

        # Unit vector representing the robot's forward direction
        robot_dx = np.cos(self.robot_yaw)
        robot_dy = np.sin(self.robot_yaw)

        for _ in range(200):  # Limit attempts to avoid infinite loops
            x_candidate = uniform(self.map_origin_x, self.map_origin_x + self.map_width * self.map_resolution)
            y_candidate = uniform(self.map_origin_y, self.map_origin_y + self.map_height * self.map_resolution)

            if self.is_valid_point(x_candidate, y_candidate):
                dx = x_candidate - self.robot_x
                dy = y_candidate - self.robot_y

                distance = np.hypot(dx, dy)
                if distance < 0.5:  # Avoid picking a point too close to the robot
                    continue

                # Normalize the candidate direction vector
                candidate_dx = dx / distance
                candidate_dy = dy / distance

                # Compute alignment score (dot product between robot's direction and candidate direction)
                alignment_score = robot_dx * candidate_dx + robot_dy * candidate_dy

                # Final score: weight forward direction higher but still consider distance
                score = alignment_score - 0.01 * distance  # Tradeoff factor

                if score > best_score:
                    best_score = score
                    best_point = (x_candidate, y_candidate)

                # Early exit if an optimal point is found
                if best_score > 0.99:  
                    break

        if best_point:
            pub_msg = PointStamped()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "map"
            pub_msg.point.x = best_point[0]
            pub_msg.point.y = best_point[1]
            pub_msg.point.z = 0.0

            self.get_logger().info(f"Publishing new goal: x={best_point[0]:.2f}, y={best_point[1]:.2f}")
            self.publisher.publish(pub_msg)
            return True

        self.get_logger().warn("Could not find a valid random point!")
        return False


def main():
    rclpy.init()
    node = RandomPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
