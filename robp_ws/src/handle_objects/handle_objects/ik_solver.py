#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#import numpy as np
import PyKDL as kdl

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        # ---- Define Robot Structure ----
        # we need to go from base link to base plate and then
        # base plate -> 1st "blue" servo -> green servo ->ID3 servo joint -> gripper rotation joint
        a,b,c,d,e,f,g = 0.01*7.4, 0.01*1.5, 0.01*11.5, 0.01*4.45, 0.01*21.3, 0.01*5.3, 0.01*9.5
        self.base_height = a  # Height of the base link
        self.link_lengths = [b, c-b, e-c-d, f]  # Lengths of the 5 arm links (excluding gripper)
        self.gripper_offset = g  # Distance from wrist to gripper center

        # ---- Create KDL Chain ----
        self.chain = kdl.Chain()

        # Define Joints and Links
        self.create_arm_chain()

        # ---- Create Solvers ----
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)  # Levenberg-Marquardt IK Solver

        # ---- Test IK with a Target Position ----
        target_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.15, 0, 0.4))  # Example target position
        #self.solve_ik(target_pose)

    def create_arm_chain(self):
        """
        Create the kinematic chain based on the robot arm structure.
        
        In this segment, we add a fixed segment from the base to the first moving joint.
        """
        # Create a frame with no rotation and a translation along the Z-axis (base height)
        frame_base = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, self.base_height))
        
        # Create a fixed joint.
        # "base_joint" is the joint name.
        # kdl.Joint.None indicates that this joint is fixed (i.e., it has no degrees of freedom).
        fixed_joint = kdl.Joint("base_joint", kdl.Joint.None)
        
        # Create a segment with a name, the fixed joint, and the transformation frame.
        self.chain.addSegment(kdl.Segment("base_to_first_joint", fixed_joint, frame_base))


        # Joint 1: Rotates around Z-axis (base rotation)
        self.add_joint(kdl.Joint.RotZ, kdl.Vector(0, 0, 0))

        # Joint 2: Rotates around Y-axis (shoulder) (1st BLUE SERVO)
        self.add_joint(kdl.Joint.RotY, kdl.Vector(0, 0, self.link_lengths[0]))

        # Joint 3: Rotates around Y-axis (elbow) (GREEN SERVO)
        self.add_joint(kdl.Joint.RotY, kdl.Vector(self.link_lengths[1], 0, 0))

        # Joint 4: Rotates around Y-axis (wrist pitch) (ID3 SERVO)
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
