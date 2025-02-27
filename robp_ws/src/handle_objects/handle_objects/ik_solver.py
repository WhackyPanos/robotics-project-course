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

        # --- Example FK
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain) 
        #self.solve_fk([-15.706226190621685, -58.89322565161596, 89.19116421034657, -83.70501671216269, -6.281438203134586, -15.707868506470371])
        self.solve_fk([0.0,90,0.0,0.0,0.0,0.0])
        self.solve_fk([0.0,90*pi/180,0.0,0.0,0.0,0.0])

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

        # --- Moving Joints --- self.link_lengths = [self.b, self.c - self.b, self.e - self.c , self.f]
        # Joint 1: Base rotation (around Z-axis)
        self.add_joint("joint1", kdl.Joint.RotZ, kdl.Vector(0, 0, self.link_lengths[0]))
        # Joint 2: 1st "blue" servo (rotates around Y-axis)
        self.add_joint("joint2", kdl.Joint.RotY, kdl.Vector(0, 0, self.link_lengths[1] ))
        # Joint 3: green servo (rotates around Y-axis)
        self.add_joint("joint3", kdl.Joint.RotY, kdl.Vector(0, 0, self.link_lengths[2]))
        # Joint 4: ID3 servo  (rotates around Y-axis)
        self.add_joint("joint4", kdl.Joint.RotY, kdl.Vector(0, 0, self.link_lengths[3]))
        # Joint 5: Wrist rotation (rotates around Z-axis)
        self.add_joint("joint5", kdl.Joint.RotZ, kdl.Vector( 0, 0, self.gripper_offset))
        # Joint 6: Gripper closing/opening (rotates around X-axis)
        self.add_joint("joint6", kdl.Joint.RotX, kdl.Vector(0, 0, 0))

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
        
    
    def solve_ik(self, target_pose, provided_initial_guess, eps=5e-3, maxiter=100000):
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

        fksol = kdl.ChainFkSolverPos_recursive(self.chain)
        iksol = kdl.ChainIkSolverVel_pinv(self.chain, eps, maxiter)  # or another IK solver depending on your needs
        
        # joints limits in the solver domain
        self.q_min = kdl.JntArray(self.chain.getNrOfJoints())
        self.q_max = kdl.JntArray(self.chain.getNrOfJoints())
        self.q_min[0] = -120.0* pi / 180 #-120
        self.q_min[1] = -120.0* pi / 180 #-60.0
        self.q_min[2] = -120.0* pi / 180 #-90.0
        self.q_min[3] = -120.0* pi / 180 #-90.0
        self.q_min[4] = -120.0* pi / 180 #-120.0
        self.q_min[5] = -1000#-1000  # Assuming the last joint has no specific limits

        self.q_max[0] = 120.0* pi / 180 #120.0
        self.q_max[1] = 120.0* pi / 180 #60.0
        self.q_max[2] = 120.0* pi / 180 #90.0
        self.q_max[3] = 120.0* pi / 180 #90.0
        self.q_max[4] = 120.0 * pi / 180 #120.0
        self.q_max[5] = 1000  # Assuming the last joint has no specific limits

        # create solver
        # self.ik_solver = kdl.ChainIkSolverPos_NR_JL(
        #     chain = self.chain,
        #     q_min = self.q_min, 
        #     q_max = self.q_max,
        #     fksolver= fksol,
        #     iksolver = iksol,
        #     eps = eps,
        #     maxiter = maxiter
        #     ) 
        
        self.ik_solver = kdl.ChainIkSolverPos_LMA(
            chain = self.chain,
            eps = eps,
            maxiter = maxiter,
            ) # Levenberg-Marquardt IK Solver

        # Determine the number of joints 
        num_joints = self.chain.getNrOfJoints()
        joint_positions = kdl.JntArray(num_joints)
        initial_guess = kdl.JntArray(num_joints)  # You can set a custom guess here if desired
        for i,angle in enumerate(provided_initial_guess):
            initial_guess[i] = angle

        result = self.ik_solver.CartToJnt(initial_guess, target_pose, joint_positions)

        if result >= 0:
            angles = [joint_positions[i] for i in range(num_joints)]
            #self.get_logger().info(f"IK Original Solution: {angles}")
        else:
            angles = None
            #self.get_logger().error("IK Solver failed!")
            self.get_logger().error(f"Result = {result}")
        return result, angles





