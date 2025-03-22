#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import subprocess
import os
import signal
from rclpy.duration import Duration

class TfMonitorNode(Node):
    def __init__(self):
        super().__init__('tf_monitor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_process = None

        # Start the static transform publisher
        self.start_static_transform()

        # Check every second if the dynamic transform exists
        self.timer = self.create_timer(1.0, self.check_tf)

    def start_static_transform(self):
        """Launch the static transform publisher"""
        self.get_logger().info("Starting static transform between map -> odom")
        self.static_tf_process = subprocess.Popen([
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "--frame-id", "map",
            "--child-frame-id", "odom"
        ])

    def check_tf(self):
        """Check if the dynamic transform is available and kill the node immediately"""
        try:
            # Attempt to get the transform with a short timeout (0.1 sec)
            transform = self.tf_buffer.lookup_transform(
                "map", "odom", rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )

            # Compare the timestamp of the transform with the current time
            now_ns = self.get_clock().now().nanoseconds
            transform_stamp_ns = transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec

            # If the transform was updated within the last second, assume it's dynamic
            if now_ns - transform_stamp_ns < 1e9:
                self.get_logger().info("Dynamic transform detected! Killing static publisher and shutting down.")
                if self.static_tf_process:
                    os.kill(self.static_tf_process.pid, signal.SIGTERM)
                    self.static_tf_process = None
                # Immediately shutdown the node
                self.timer.cancel()
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().info("Waiting for dynamic transform...")

def main():
    rclpy.init()
    node = TfMonitorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
