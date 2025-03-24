#!/usr/bin/env python

# Software License Agreement (BSD License)
# 
# Copyright (c) 2024, LAAS-CNRS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Simon WASIELA 

# importing sys
import sys
import os

# Get the current directory of this script file (script.py), which is /CAMP/src/robots/scripts
current_dir = os.path.dirname(__file__)
parent_dir1 = os.path.abspath(os.path.join(current_dir, '..'))
parent_dir2 = os.path.abspath(os.path.join(current_dir, '../../../../devel/lib/python3/dist-packages'))
# adding Scripts to the system path
sys.path.insert(0, parent_dir1)
sys.path.insert(0, parent_dir2)

import rospy
import random
import numpy as np
from math import *
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
from gazebo_msgs.msg import LinkState 
from attach_links_genom import AttachLinks
from scipy.spatial.transform import Rotation as rot
from tf.transformations import euler_from_quaternion, quaternion_matrix

len_cyl = 0.005

def checkInRing(radius_sq, ring_center, testpt):
    if abs(ring_center[0]-testpt[0]) < len_cyl and abs(ring_center[1]-testpt[1]) < radius_sq and abs(ring_center[2]-testpt[2]) < radius_sq:
        return True
    else:
        return False

# ----------------------------------------- THE SCENARIO ------------------------------------------ #
ring_radius = 0.02
perch_length = 0.5
attach_length = 0.40
z_eef = -0.0875


rings = []
ring_pos_0 = [0.0, -0.8, 1.3, 0.0] # X Y Z Yaw(rad) 
ring_pos_1 = [2.6, 0.2, 1.2, 1.5708] # X Y Z Yaw(rad) 
ring_pos_2 = [-0.5, 1.1, 1.3, 3.1415] # X Y Z Yaw(rad) 
ring_pos_3 = [-2.37, 0.75, 1.4, -1.5708] # X Y Z Yaw(rad) 
rings.append(ring_pos_0)
rings.append(ring_pos_1)
rings.append(ring_pos_2)
rings.append(ring_pos_3)

ring_attached = []

link_name = "quad::base"  # Replace with the actual link name
reference_frame = "world"

# Initialize the ROS node 
rospy.init_node('Attacher', anonymous=True)

# ----------------------------------------- THE ATTACHER ------------------------------------------ #
attacher = AttachLinks()

# ----------------------------------------- TO GET THE CURRENT POSE ------------------------------------------ #
# Create a client for the GetLinkState service
rospy.wait_for_service('/gazebo/get_link_state')
get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
# Create a request message
request = GetLinkStateRequest()
request.link_name = link_name
request.reference_frame = reference_frame
# Your loop code
rate = rospy.Rate(250)  # Adjust the loop frequency as needed (e.g., 200 Hz)

# ----------------------------------------- THE LOOP ------------------------------------------ #
print("Wainting to start...")
while not rospy.is_shutdown():
    
    try:
        # Call the GetLinkState service
        response = get_link_state_service(request)
        
        if response.success:
            link_state = response.link_state
            
            x = link_state.pose.position.x
            y = link_state.pose.position.y
            z = link_state.pose.position.z
            qx = link_state.pose.orientation.x
            qy = link_state.pose.orientation.y 
            qz = link_state.pose.orientation.z
            qw = link_state.pose.orientation.w
            
            quat = [qx, qy, qz, qw]
            # euler_angles = euler_from_quaternion(quat)
            
            # Convert the quaternion to a rotation matrix
            rotation_matrix = quaternion_matrix(quat)

            # Extract the 3x3 rotation matrix part (ignoring translation)
            R = rotation_matrix[:3, :3]
            
            # Check if we have passed through a ring and attach it
            M = np.zeros((4,4))
            M[0:3,0:3] = R
            M[0,3] = x
            M[1,3] = y
            M[2,3] = z
            M[3,3] = 1.0
            
            vec_eef_local = np.zeros((4,1))
            vec_eef_local[0,0] = perch_length * cos(0.785398)
            vec_eef_local[1,0] = perch_length * sin(0.785398)
            vec_eef_local[2,0] = z_eef
            vec_eef_local[3,0] = 1.0
            
            eef_global = np.dot(M,vec_eef_local)

            eef_x = eef_global[0]
            eef_y = eef_global[1]
            eef_z = eef_global[2]    
            
            for i in range(len(rings)):
                ring_pos = rings[i]
                if checkInRing(ring_radius, ring_pos, [eef_x, eef_y, eef_z]):
                    if i in ring_attached:
                        continue
                    # Set the bar to the desired position
                    link_msg = LinkState()
                    vec_eef_attached = np.zeros((4,1))
                    vec_eef_attached[0,0] = attach_length * cos(0.785398)
                    vec_eef_attached[1,0] = attach_length * sin(0.785398)
                    vec_eef_attached[2,0] = z_eef-0.04
                    vec_eef_attached[3,0] = 1.0
                    
                    eef_global_attached = np.dot(M,vec_eef_attached)
            
                    link_msg.pose.position.x = eef_global_attached[0]
                    link_msg.pose.position.y = eef_global_attached[1]
                    link_msg.pose.position.z = eef_global_attached[2]
                    roll = 0.0
                    pitch = 0.0
                    yaw = ring_pos[3]
                    link_msg.pose.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                    link_msg.pose.orientation.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
                    link_msg.pose.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
                    link_msg.pose.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                    # Attach the bar
                    attacher.attach(link_msg, i+1)
                    ring_attached.append(i)
        else:
            rospy.logerr("Failed to get link state")
        
        # Sleep to control the loop rate
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        
    rate.sleep()
print("Shutdown !")
