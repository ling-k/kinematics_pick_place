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
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np



## convert a sympy matrix to a numpy
def sym_num(sym):
    
    return np.array(sym.tolist()).astype(np.float64)
    


# calculate the rotation matrix from the base to the end gripper:  ROT * Rot_correct
def rpyToRotation(r, p, y):
    
    ROT = Matrix([
                 [cos(p)*cos(y), sin(p)*sin(r)*cos(y) - sin(y)*cos(r), sin(p)*cos(r)*cos(y) + sin(r)*sin(y)],
                 [sin(y)*cos(p), sin(p)*sin(r)*sin(y) + cos(r)*cos(y), sin(p)*sin(y)*cos(r) - sin(r)*cos(y)],
                 [      -sin(p),                        sin(r)*cos(p),                        cos(p)*cos(r)]])

    Rot_correct = Matrix([
                          [0.,   0., 1.0],
                          [0., -1.0,  0.],
                          [1.0,  0.,  0.]])

    ROT = ROT * Rot_correct
    
    return sym_num(ROT)




 ##  Get the rotation matrix from base to WC, using q1, q2, q3
def eval_r0_3(q1, q2, q3):
    
    
    R0_3_eval = Matrix([
           [-sin(q3)*sin(q2 - 0.5*pi)*cos(q1) + cos(q1)*cos(q3)*cos(q2 - 0.5*pi), -sin(q3)*cos(q1)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q1)*cos(q3), -sin(q1)],
           [-sin(q1)*sin(q3)*sin(q2 - 0.5*pi) + sin(q1)*cos(q3)*cos(q2 - 0.5*pi), -sin(q1)*sin(q3)*cos(q2 - 0.5*pi) - sin(q1)*sin(q2 - 0.5*pi)*cos(q3),  cos(q1)],
           [                -sin(q3)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q3),                  sin(q3)*sin(q2 - 0.5*pi) - cos(q3)*cos(q2 - 0.5*pi),        0]])
    
    return sym_num(R0_3_eval) 



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:


        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

            ROT_EE = rpyToRotation(roll, pitch, yaw) 
            
            # calculate the wrist center  
            EE = [px,py,pz]    
            WC = EE - (0.303) * ROT_EE[:, 2]
            
            # calculate joint angles using Geometric IK method 
            theta1 = atan2(WC[1], WC[0])
            
            side_a =  1.501
            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            side_c = 1.25
            
            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
            
            theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi / 2 - (angle_b + 0.036)
            
            # calculate the rotation matrix from base to link 3 
            R0_3 = eval_r0_3(theta1, theta2, theta3)
            
            R3_6 = np.dot(np.linalg.inv(R0_3), ROT_EE)
            
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
