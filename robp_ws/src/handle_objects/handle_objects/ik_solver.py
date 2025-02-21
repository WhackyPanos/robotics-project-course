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
        #self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)  # Levenberg-Marquardt IK Solver
        self.ik_solver = kdl.ChainIkSolverPos(self.chain)

        # Set maximum iterations (default is 50) and  set error tolerance (default is 1e-3)
        #self.ik_solver.setMaxIterations(100)
        #self.ik_solver.setTolerance(1e-1)

        # --- Example FK
        self.solve_fk([0,90*pi/180, 20*pi/180])

        # ---- Example IK Call (uncomment to test) ----
        target_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(19.208987*0.01, 0, 5.5482*0.01))
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
        


    def J(self, lisT):
        J = np.zeros((6, 7)) #initiate jacobian matrix
        #auxiliary vectors to extract desired z_i-1 & p_i-1 
        z0 = np.array([0, 0, 1])      
        p0 = np.array([0, 0, 0, 1])

        for i in range(7):
            #compute z_i-1
            if i==0: 
                z= z0
            else:
                z = np.matmul(lisT[i-1][:3,:3], z0)
            #compute p_i-1
            p = np.matmul(lisT[i-1], p0)[:3]    
            #compute pe
            pe = np.matmul(lisT[6], p0)[:3]
            Jp = np.cross(z, pe-p)
            Jo = z
            J[:, i] = np.concatenate((Jp, Jo))
        return J


    def T(self,alpha, d,a,theta):
        Rot_theta = np.array([[cos(theta), -sin(theta), 0, 0],
                            [sin(theta), cos(theta) , 0, 0],
                            [0,0,1,0],
                            [0,0,0,1]])
        
        Rot_alpha = np.array([[1,0,0,0],
                            [0, cos(alpha), -sin(alpha) , 0],
                            [0, sin(alpha), cos(alpha)  , 0],
                            [0,0,0,1]])
        
        Trans_d = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,d],
                        [0,0,0,1]])
        
        Trans_a = np.array([[1,0,0,a],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        
        T = np.matmul(np.matmul(Rot_theta,Trans_d), np.matmul(Rot_alpha,Trans_a))
        return T
    
    def transforms(self,q):
    
        # Initiate constants
        h1 = 0.311
        h2 = 0.078
        L = 0.4
        M = 0.39

        parametersDH = np.array([[np.pi/2, self.a + self.b, 0, q[0]], # was h1 before
                                [-np.pi/2, 0, 0, q[1]],
                                [-np.pi/2, L, 0,q[2] ],
                                [np.pi/2, 0, 0, q[3]],
                                [np.pi/2, M, 0, q[4]],
                                [-np.pi/2, 0, 0, q[5]],
                                [ 0, h2, 0, q[6]]])

        multipT=np.identity(4) 
        lisT=[] # initiate list with the necessary transforms

        for i in range(7):
            alpha = parametersDH[i, 0]
            d = parametersDH[i, 1]
            a = parametersDH[i, 2]
            theta = parametersDH[i, 3]
            t = self.T(alpha, d,a,theta)
            multipT = np.matmul(multipT,t)
            lisT.append(np.array(multipT))  

        return multipT, lisT


    def poseError(self,Rd,T,x,y,z):

        Rd = np.array(Rd)
        Re = T [:3, :3]
        actualX = T[:3, 3]
        
        # apply formulas from the book -> extract vectors from Rd & Re to perform cross product
        n_e = Re[:, 0] 
        s_e = Re[:, 1] 
        a_e = Re[:, 2]  
        n_d = Rd[:, 0]
        s_d = Rd[:, 1]
        a_d = Rd[:, 2]

        e_xPos = np.array([x - actualX[0], y - actualX[1], z - actualX[2]])
        e_xOri = 0.5 * (np.cross(n_e, n_d)+ np.cross(s_e, s_d)+ np.cross(a_e, a_d))

        e_X = np.concatenate((e_xPos, e_xOri)) 

        return e_X
    
    def solve_ik_mine(self,point, R, joint_positions):

        x = point[0]
        y = point[1]
        z = point[2]
        q = joint_positions #it must contain 7 elements

        q = np.array(joint_positions)

        epsilon = 10**-3 # tolerance threshold 
        e_X = 1 #initiate error to start loop
    
        # loop to numerically compute q
        while np.linalg.norm(e_X) > epsilon:
            finalT, lisT = self.transforms(q)
            j = self.J(lisT)  # Compute the Jacobian in the base frame
            e_X = self.poseError(R, finalT, x, y, z)
            #print(np.linalg.norm(e_X))
            inv_J = np.linalg.pinv(j) 
            e_q = np.matmul(inv_J, e_X)
            q += e_q

        return q

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
            self.get_logger().error("IK Solver failed!")
            self.get_logger().error(f"Result = {result}")






