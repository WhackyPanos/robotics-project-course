import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.publisher_ = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        #self.timer = self.create_timer(1.0, self.publish_servo_command) 
        self.publish_servo_command()
        #publish_servo_command()

    def publish_servo_command(self):
        self.get_logger().info("timer has fired")
        
        msg = Int16MultiArray()

        # Setting up the exact layout structure
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="", size=0, stride=0)],  # Matches the empty label, size=0, stride=0
            data_offset=0
        )

        # Servo positions exactly as specified       
        msg.data = [-1, -1, 2000, 13000, 1000, -1, 1000, 1000, 1000, 1000, 1000, 1000] # go get the cube
        self.get_logger().info(f'Attempting to publish: {msg.data}')
        self.publisher_.publish(msg)
        time.sleep(2)
        msg.data = [11000, -1, -1, -1, -1, -1, 500, 500, 500, 500, 500, 500] # close gripper
        self.get_logger().info(f'Attempting to publish: {msg.data}')
        self.publisher_.publish(msg)
        time.sleep(2)
        msg.data = [-1, 12000, 12000, 12000, 12000, -1, 1000, 1000, 1000, 1000, 1000, 1000]
        #msg.data = [10000, -1, 2000, 13000, 1000, -1, 2000, 2000, 2000, 2000, 2000, 2000] # go get the cube
        self.get_logger().info(f'Attempting to publish: {msg.data}')
        self.publisher_.publish(msg)
        self.get_logger().info(f'Message published successfully.')
        timer_period = 0.25
        #self.timer = self.create_timer(timer_period, self.callback)

        """
        ros2 topic pub /multi_servo_cmd_sub --once std_msgs/Int16MultiArray "{layout: {dim: 
        [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]}"

        The data component of the message contains the desired servo positions and the move time. It should always contain 12 integer entries. 
        To be more clear it is structured as follows:
        data:[servo1_desired_pos, servo2_desired_pos, ... , servo6_desired_pos, servo1_move_time, ... , servo6_move_time]. 
        
        In this particular case, servos 2-6 are set to their middle position (120 deg) and the arm moves upright in 500 ms.
        """

    def callback(self):
        self.get_logger().info("timer has fired")

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()