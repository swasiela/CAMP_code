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

import sys
import os

# Get the current directory of this script file (script.py), which is /CAMP/src/robots/scripts
current_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
  
# adding Scripts to the system path
sys.path.insert(0, parent_dir)

from services_msgs.srv import DynamicSrv
# from models.quadrotor_model_classic_dynamic import *
from models.quadrotor_model_CoM_dynamic import *
import rospy

def add_dynamic_server():
    
    rospy.init_node('quadrotor_dynamic_server')

    # Initialise Jitcode model
    model = Jetson(6)
    model.generate(3, verbose=True, overwrite=False)
    model.ODE.set_integrator('dopri5')

    # Initialise service
    s = rospy.Service('compute_dynamic', DynamicSrv, model.compute_dynamic)
    print("Ready to compute dynamic.")
    rospy.spin()

if __name__ == "__main__":
    add_dynamic_server()
