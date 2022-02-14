#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import *
import numpy as np
import math


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print ("No valid poses received")
        return (-1)
    else:

    # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	# Create Modified DH parameters
        s = {alpha0:     0, a0:      0, d1:  0.75, 
             alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2 - np.pi/2,
             alpha2:     0, a2:   1.25, d3:     0, 
             alpha3: -pi/2, a3: -0.054, d4:  1.50, 
             alpha4:  pi/2, a4:      0, d5:     0, 
             alpha5: -pi/2, a5:      0, d6:     0, 
             alpha6:     0, a6:      0, d7: 0.303, q7: 0}
	# Define Modified DH Transformation matrix
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a1],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])
        T0_1 = T0_1.subs(s)
        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])
        T1_2 = T1_2.subs(s)
        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])
        T2_3 = T2_3.subs(s)
        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])
        T3_4 = T3_4.subs(s)
        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s)
        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s)
        T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
        T6_7 = T6_7.subs(s)
	# Extract rotation matrices from the transformation matrices
        R0_1 = T0_1[:3,:3]
        R1_2 = T1_2[:3,:3]
        R2_3 = T2_3[:3,:3]
        R3_4 = T3_4[:3,:3]
        R4_5 = T4_5[:3,:3]
        R5_6 = T5_6[:3,:3]
        R6_7 = T6_7[:3,:3]

    # Initialize service response
        joint_trajectory_list = []
        for x in range(0, len(req.poses)):
	# Extract end-effector position and orientation from request
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            (roll, pitch, yaw) = euler_from_quaternion(
                req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w)
	# Compensate for rotation discrepancy between DH parameters and Gazebo
        R_y = Matrix([[ cos(-np.pi/2),     0,  sin(-np.pi/2)],
                      [             0,     1,              0],
                      [-sin(-np.pi/2),     0,  cos(-np.pi/2)]])

        R_z = Matrix([[ cos(np.pi),   -sin(np.pi),       0],
                      [ sin(np.pi),    cos(np.pi),       0],
                      [          0,             0,       1]])
        R_corr = simplify(R_z * R_y)
    # Compute R0_7
        R_x = Matrix([[ 1,        0,           0],
                      [ 0,  cos(roll),    -sin(roll)],
                      [ 0,  sin(roll),    cos(roll)]])

        R_y = Matrix([[ cos(pitch),    0,  sin(pitch)],
                      [ 0,          1,        0],
                      [ -sin(pitch),   0,  cos(pitch)]])

        R_z = Matrix([[cos(yaw), -sin(yaw),      0],
                      [sin(yaw),  cos(yaw),      0],
                      [       0,         0,      1]])
    # Calculate joint angles using Geometric IK method
        R0_7 = R_z * R_y * R_x * R_corr # Tait-Bryon X-Y-Z extrinsic rotations
    # Calculate Wrist center position
        Pos = Matrix([[px],[py],[pz]])
        W = Pos - d7 * R0_7[:,2]
        W = W.subs(s)
        Wx = W[0,0]
        Wy = W[1,0]
        Wz = W[2,0]
    # Calculate the A,B and C parameters
        A = 1.50
        B = sqrt(Wx**2,(Wz - 0.75)**2)
        B = sqrt((sqrt(Wx**2+Wy**2)-0.35)**2+(Wz-0.75)**2)
        C = 1.25
    # Calculate the first three joint angles
        theta1 = atan2(Wy,Wx).subs({pi:np.pi})
        theta2 = (pi/2 - acos((B**2 + C**2 - A**2)/(2*B*C)) - atan2(Wz - 0.75,sqrt(Wx**2 + Wy**2) - 0.35)).subs({pi:np.pi})
        theta3 = (pi/2 - (acos((A**2 + C**2 - B**2)/(2*A*C)) + 0.036)).subs({pi:np.pi}) 
    # Calculate the rotation matrix R3_6
        R0_3 = R0_1 * R1_2 * R2_3
        R0_3 = R0_3.subs(s)
        R0_3 = R0_3.subs({q1:theta1,q2:theta2,q3:theta3})
        R3_7 = R0_3.T * R0_7
        theta4 = atan2(R3_7[2,2],-R3_7[0,2])
        theta5 = atan2(sqrt(R3_7[2,2]**2 + R3_7[0,2]**2),R3_7[1,2])
        theta6 = atan2(-R3_7[1,1],R3_7[1,0])
    # Send the values of the angles
	    joint_trajectory_point.positions = [float(theta1), float(theta2), float(theta3), float(theta4), float(theta5), float(theta6)]
	    joint_trajectory_list.append(joint_trajectory_point)
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        
        return (CalculateIKResponse(joint_trajectory_list))


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print ("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
