import rclpy
import rclpy.duration
from rclpy.node import Node

import rclpy.time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class Mapping(Node):
    def __init__(self):
        super().__init__('mapping')
        
        # The lidar scans will be publsihed as a point cloud on the /map topic
        self.publisher = self.create_publisher(PointCloud2, '/map', 10) 
        # The listener_callback is called every time a new LaserScan msg is published from the lidar
        self.subscription = self.create_subscription(LaserScan,'/scan',self.listener_callback,10)
        # Initialize the transform buffer for the listener
        self.tf_buffer = Buffer()
        # Initialize the transform
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Initializes the projection 
        self.proj = LaserProjection() # Converts a LaserScan to a PointCloud2

        self.N = 10
        self.counter = 0

    def listener_callback(self, msg):
            self.counter += 1 
            if self.counter < self.N:
                return
            self.counter = 0

            to_frame_rel = 'map'
            from_frame_rel = msg.header.frame_id # Lidar link
            time = rclpy.time.Time().from_msg(msg.header.stamp)

            ### BONUS BELOW - uncomment to test ###

            # # Wait for the transform asynchronously
            # tf_future = self.tf_buffer.wait_for_transform_async(
            #     target_frame=to_frame_rel,
            #     source_frame=from_frame_rel,
            #     time=time
            # )

            # # Spin until transform found or `timeout_sec` seconds has passed
            # rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

            ### BONUS ABOVE - uncomment to test ###

            # Lookup the transform
            try:
                t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time)
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return

            # Project lidar to point cloud
            cloud = self.proj.projectLaser(msg)

            # Transform point cloud
            cloud_out = do_transform_cloud(cloud, t)

            self.publisher.publish(cloud_out)

def main():
    rclpy.init()
    node = Mapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    