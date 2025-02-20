#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import PyKDL as kdl

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        # ---- Define Robot Structure (Modify These for Your Robot) ----
        self.link_lengths = [0.1, 0.2, 0.2, 0.1, 0.05]  # Lengths of the 5 arm links (excluding gripper)
        self.base_height = 0.1  # Height of the base link
        self.gripper_offset = 0.05  # Distance from wrist to gripper center

        # ---- Create KDL Chain ----
        self.chain = kdl.Chain()

        # Define Joints and Links
        self.create_arm_chain()

        # ---- Create Solvers ----
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)  # Levenberg-Marquardt IK Solver

        # ---- Test IK with a Target Position ----
        target_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.3, 0.2, 0.4))  # Example target position
        self.solve_ik(target_pose)

    def create_arm_chain(self):
        """
        Create the kinematic chain based on the robot arm structure.
        """
        # Base to first joint (height offset)
        frame_base = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, self.base_height))
        self.chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint), frame_base))

        # Joint 1: Rotates around Z-axis (base rotation)
        self.add_joint(kdl.Joint.RotZ, kdl.Vector(0, 0, 0))

        # Joint 2: Rotates around Y-axis (shoulder)
        self.add_joint(kdl.Joint.RotY, kdl.Vector(0, 0, self.link_lengths[0]))

        # Joint 3: Rotates around Y-axis (elbow)
        self.add_joint(kdl.Joint.RotY, kdl.Vector(self.link_lengths[1], 0, 0))

        # Joint 4: Rotates around Y-axis (wrist pitch)
        self.add_joint(kdl.Joint.RotY, kdl.Vector(self.link_lengths[2], 0, 0))

        # Joint 5: Rotates around Z-axis (wrist rotation)
        self.add_joint(kdl.Joint.RotZ, kdl.Vector(self.link_lengths[3], 0, 0))

        # Joint 6 (Gripper orientation control): Rotates around X-axis
        self.add_joint(kdl.Joint.RotX, kdl.Vector(0, 0, self.gripper_offset))

    def add_joint(self, joint_type, translation):
        """
        Add a joint and link segment to the KDL chain.
        :param joint_type: The type of joint (e.g., RotX, RotY, RotZ).
        :param translation: A kdl.Vector representing the link translation.
        """
        joint = kdl.Joint(joint_type)
        frame = kdl.Frame(kdl.Rotation.Identity(), translation)
        self.chain.addSegment(kdl.Segment(joint, frame))

    def solve_ik(self, target_pose):
        """
        Solve inverse kinematics for a given end-effector target pose.
        :param target_pose: The desired position and orientation of the end-effector.
        It assumes the format target_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 1.57), kdl.Vector(0.3, 0.2, 0.4)) -> roll,pitch,yaw and x,y,z
        These values should be in the robot base frame
        """
        joint_positions = kdl.JntArray(5)  # We solve for 5 joints (excluding gripper open/close)
        result = self.ik_solver.CartToJnt(kdl.JntArray(5), target_pose, joint_positions)

        if result >= 0:
            angles = [joint_positions[i] for i in range(5)]
            self.get_logger().info(f"IK Solution: {angles}")
        else:
            self.get_logger().error("IK Solver failed!")

"""
def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    node.destroy_node()  # Cleanup after execution
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
