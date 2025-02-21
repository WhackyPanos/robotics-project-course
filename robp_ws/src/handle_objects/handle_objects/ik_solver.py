#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import PyKDL as kdl
from math import pi
import numpy as np
from math import cos, sin


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        # ---- Define Robot Structure ----
        # we need to go from base link to base plate and then
        # base plate -> 1st "blue" servo -> green servo ->ID3 servo 
        # Dimensions in meters
        self.a, self.b, self.c, self.d, self.e, self.f, self.g = 0.01 * 7.4, 0.01 * 1.5, 0.01 * 11.5, 0.01 * 4.45, 0.01 * 21.3, 0.01 * 5.3, 0.01 * 9.5
        self.base_height = self.a             # Height of the base link
        self.link_lengths = [self.b, self.c - self.b, self.e - self.c , self.f]  # Lengths for the moving segments
        self.gripper_offset = self.g          # Distance from wrist to gripper center

        # ---- Create KDL Chain ----
        self.chain = kdl.Chain()
        self.create_arm_chain()

        # ---- Create Solvers ----
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_vel_solver = kdl.ChainIkSolverVel_pinv(self.chain)

        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain, eps=5e-3, maxiter=100000) # Levenberg-Marquardt IK Solver
        # self.ik_solver = kdl.ChainIkSolverPos_NR(
        #     self.chain,
        #     self.fk_solver,
        #     self.ik_vel_solver,
        #     maxiter=1000,
        #     eps=0.5)

        # --- Example FK
        #self.solve_fk([0,20*pi/180, 20*pi/180])
        self.solve_fk([0.0, 0.6147484289969556, -0.16042951294421512])

        # ---- Example IK Call (uncomment to test) ----
        target_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(9.963456717271555*0.01, 0, 25.932833880796892*0.01))
        self.solve_ik(target_pose)


    def create_arm_chain(self):
        """
        Build the kinematic chain representing your robot arm.
        
        Each segment of the chain is defined by:
          - A joint (which may be fixed or moving)
          - A frame (kdl.Frame) that represents the static transformation from the previous segment.
        
        In this example:
          1. A fixed segment from the base to the first joint.
          2. Then six moving joints representing the arm's DOFs.
        """

        # --- Fixed Base Segment ---
        # Create a frame: no rotation and a translation along Z (base height)
        frame_base = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, self.base_height))
        # Create an explicit fixed joint (DOF = 0) with a name
        fixed_joint = kdl.Joint("base_joint")
        # Create and add the fixed segment with a name, fixed_joint, and frame_base
        self.chain.addSegment(kdl.Segment("base_to_first_joint", fixed_joint, frame_base))

        # --- Moving Joints ---
        # Joint 1: Base rotation (around Z-axis)
        self.add_joint("joint1", kdl.Joint.RotZ, kdl.Vector(0, 0, self.link_lengths[0]))
        # Joint 2: 1st "blue" servo (rotates around Y-axis)
        self.add_joint("joint2", kdl.Joint.RotY, kdl.Vector(0, 0, self.link_lengths[1] ))
        # Joint 3: green servo (rotates around Y-axis)
        self.add_joint("joint3", kdl.Joint.RotY, kdl.Vector(0, 0, self.link_lengths[2]))
        # Joint 4: ID3 servo  (rotates around Y-axis)
        #self.add_joint("joint4", kdl.Joint.RotY, kdl.Vector(0, 0, self.link_lengths[3]))
        # Joint 5: Wrist rotation (rotates around Z-axis)
        #self.add_joint("joint5", kdl.Joint.RotZ, kdl.Vector( 0, 0, self.link_lengths[3]))
        # Joint 6: Gripper closing/opening (rotates around X-axis)
        #self.add_joint("joint6", kdl.Joint.RotX, kdl.Vector(0, 0, self.gripper_offset))

    def add_joint(self, name, joint_type, translation):
        """
        Add a moving joint and its link (segment) to the kinematic chain.
        
        :param name: A string representing the joint (and segment) name.
        :param joint_type: The type of joint motion.
            For example: kdl.Joint.RotX, kdl.Joint.RotY, kdl.Joint.RotZ.
        :param translation: A kdl.Vector that represents the fixed translation from
            the previous joint to this joint's frame.
        
        This function creates:
          - A joint with the given name and type.
          - A frame with an identity rotation and the specified translation.
          - A segment that encapsulates the joint and its transformation.
        """
        joint = kdl.Joint(name, joint_type)
        frame = kdl.Frame(kdl.Rotation.Identity(), translation)
        self.chain.addSegment(kdl.Segment(name + "_segment", joint, frame))

    def solve_fk(self, joint_angles):
        """
        Compute forward kinematics to determine the end-effector position.
        :param joint_angles: A list of joint angles (length must match the number of joints).
        :return: The end-effector position (x, y, z) and orientation (as a rotation matrix).
        """
        if len(joint_angles) != self.chain.getNrOfJoints():
            self.get_logger().error(f"Incorrect number of joint angles provided. Gave {len(joint_angles)}, should be {self.chain.getNrOfJoints()}")
            return None

        joint_positions = kdl.JntArray(len(joint_angles))
        for i, angle in enumerate(joint_angles):
            joint_positions[i] = angle

        end_frame = kdl.Frame()
        end_frame = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, self.link_lengths[1]))
        result = self.fk_solver.JntToCart(joint_positions, end_frame)

        if result >= 0:
            pos = end_frame.p  # Position
            rot = end_frame.M  # Rotation matrix
            self.get_logger().info(f"FK Solution: Position = ({pos.x()*100}, {pos.y()*100}, {pos.z()*100}) cm")
            return pos, rot
        else:
            self.get_logger().error("FK Solver failed!")
            return None
        
    
    def solve_ik(self, target_pose):
        """
        Solve inverse kinematics for a given end-effector target pose.
        
        :param target_pose: A kdl.Frame representing the desired pose.
            - The frame contains both a rotation (orientation) and a translation (position).
            For example: kdl.Frame(kdl.Rotation.RPY(0, 0, 1.57), kdl.Vector(0.3, 0.2, 0.4))
            This pose must be expressed in the robot's base frame.
        
        Procedure:
          1. Determine the number of moving joints from the chain.
          2. Create an initial guess and an output JntArray of that size.
          3. Call the IK solver (CartToJnt) with the initial guess, target pose, and output array.
          4. Report the solution if successful, or log an error otherwise.
        """
        # Determine the number of joints (should be 6 in this case)
        num_joints = self.chain.getNrOfJoints()
        joint_positions = kdl.JntArray(num_joints)
        initial_guess = kdl.JntArray(num_joints)  # You can set a custom guess here if desired
        initial_guess[0] = 0  # Joint 1
        initial_guess[1] = (90 - 5) * pi / 180  # Joint 2
        initial_guess[2] = (20 - 5) * pi / 180  # Joint 3

        self.get_logger().info(f"For the {num_joints} joitns, the following initial configuration is: {joint_positions}")
        self.get_logger().info(f"Links lengths {self.link_lengths}")
        self.get_logger().info(f"Initial guess {initial_guess}")

        result = self.ik_solver.CartToJnt(initial_guess, target_pose, joint_positions)

        if result >= 0:
            angles = [joint_positions[i] for i in range(num_joints)]
            self.get_logger().info(f"IK Solution: {angles}")
        else:
            angles = None
            self.get_logger().error("IK Solver failed!")
            self.get_logger().error(f"Result = {result}")
        
        return angles





