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
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState, SetLinkProperties, SetLinkPropertiesRequest
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class AttachLinks():
    def __init__(self):
        
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.detach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")
        self.set_link_pose_srv = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        self.set_link_pose_srv.wait_for_service()
        self.set_link_properties_srv = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
        self.set_link_properties_srv.wait_for_service()
        print("AttachLinks intialized !")
            
    def attach(self, link_msg, ring_id):
        
        link_msg.link_name = 'ring_' + str(ring_id)
        self.set_link_pose_srv.call(link_msg)
        # Link them
        rospy.loginfo("Attaching ring and quad")
        req = AttachRequest()
        req.model_name_1 = "quad"
        req.link_name_1 = "quad::base"
        req.model_name_2 = "rings"
        req.link_name_2 = "ring_" + str(ring_id)
        self.attach_srv.call(req)
        
        link_properties_req = SetLinkPropertiesRequest()
        link_properties_req.link_name = "ring_" + str(ring_id)
        link_properties_req.gravity_mode = True
        self.set_link_properties_srv.call(link_properties_req)
    
    def detach(self, link_msg, ring_id):
        
        link_msg.link_name = 'ring_' + str(ring_id)
        self.set_link_pose_srv.call(link_msg)
        rospy.loginfo("Detaching bar and quad")
        req = AttachRequest()
        req.model_name_1 = "quad"
        req.link_name_1 = "quad::base"
        req.model_name_2 = "rings"
        req.link_name_2 = "ring_" + str(ring_id)
        self.detach_srv.call(req)
    
    def reset_detach(self, ring_id):
        rospy.loginfo("Detaching ring and quad")
        req = AttachRequest()
        req.model_name_1 = "quad"
        req.link_name_1 = "quad::base"
        req.model_name_2 = "rings"
        req.link_name_2 = "ring_" + str(ring_id)
        self.detach_srv.call(req)
        link_properties_req = SetLinkPropertiesRequest()
        link_properties_req.link_name = "ring_" + str(ring_id)
        link_properties_req.gravity_mode = False
        self.set_link_properties_srv.call(link_properties_req)
        
        