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
  
# adding Scripts to the system path
sys.path.insert(0, 'CAMP/src/genom_simu/src/scripts')

import rospy
import random
import datetime
import time
import numpy as np
from math import *
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
from gazebo_msgs.msg import LinkState 
from scipy.spatial.transform import Rotation as rot
from tf.transformations import euler_from_quaternion, quaternion_matrix

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

link_name = "quad::base"  # Replace with the actual link name
reference_frame = "world"

# Initialize the ROS node 
rospy.init_node('Traj_logger', anonymous=True)

# ----------------------------------------- TO GET THE CURRENT POSE ------------------------------------------ #
# Create a client for the GetLinkState service
rospy.wait_for_service('/gazebo/get_link_state')
get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
# Create a request message
request = GetLinkStateRequest()
request.link_name = link_name
request.reference_frame = reference_frame
# Your loop code
rate = rospy.Rate(200)  # Adjust the loop frequency as needed (e.g., 200 Hz)

# ----------------------------------------- THE LOOP ------------------------------------------ #

file_log = "CAMP/src/Results/genom_gazebo_traj.txt"
file = open(file_log,"w") 

start_time = datetime.datetime.now()
while not rospy.is_shutdown():
    
    try:
        # Call the GetLinkState service
        response = get_link_state_service(request)
        
        if response.success:
            current_time = datetime.datetime.now()
            time = (current_time - start_time).total_seconds()
            link_state = response.link_state
            
            x = link_state.pose.position.x
            y = link_state.pose.position.y
            z = link_state.pose.position.z
            qx = link_state.pose.orientation.x
            qy = link_state.pose.orientation.y 
            qz = link_state.pose.orientation.z
            qw = link_state.pose.orientation.w 
            
            line = ";"+str(time)+";"+str(x)+";"+str(y)+";"+str(z)+";"+str(qw)+";"+str(qx)+";"+str(qy)+";"+str(qz)+"\n"
            file.write(line)
            
        else:
            rospy.logerr("Failed to get link state")
        
        # Sleep to control the loop rate
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        
    rate.sleep()

file.close()
print("Shutdown !")
