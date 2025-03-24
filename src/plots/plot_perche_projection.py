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

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
import os
from math import *
from scipy.spatial.transform import Rotation as rot
from tf.transformations import euler_from_quaternion, euler_matrix

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))

LOG_GENOM = True

ring_radius = 0.02
perch_length = 0.5
perche_anlge = 0.785398
z_eef = -0.0884

y_target = -0.412 
z_target = 1.218

relative_prefix = os.path.join('src', 'results', 'RAL2024', 'Rings', 'No_opti', 'Exp_2_rings', 'Traj_')
prefix = os.path.join(camp_dir, relative_prefix)

X = []
Y = []
Z = []
Roll = []
Pitch = []
Yaw = []

Idx_opti = [2151,2378,2228,2085,2230,1945,2090,2462,2225,3072]
Idx_no_opti = [2046,1948,1974,2058,1997,1950,2047,2197,2187,2311]

# READING TRAJ FILE
###############################################################################################
for j in range(0,10):
    X_meas = []
    Y_meas = []
    Z_meas = []
    Roll_meas = []
    Pitch_meas = []
    Yaw_meas = []
    fl_name = prefix+str(j)+"/pom.log"
    file = open(fl_name,"r") 
    line = file.readlines()

    for i in range(len(line)):
        if i == 0:
            continue
        else:
            if LOG_GENOM:
                line_comps = (line[i].split(" "))
                compt_add = 0
                for elem in line_comps:
                    try:
                        number = float(elem)
                        if compt_add == 7: 
                            X_meas.append(number)
                        elif compt_add == 8:   
                            Y_meas.append(number)
                        elif compt_add == 9:   
                            Z_meas.append(number)
                        elif compt_add == 10:   
                            Roll_meas.append(number)
                        elif compt_add == 11:   
                            Pitch_meas.append(number)
                        elif compt_add == 12:   
                            Yaw_meas.append(number)
                        compt_add +=1
                    except ValueError:
                        print(f"Ignoring non-numeric value: {elem}")
            else:
                line_comps = (line[i].split(","))
                X_meas.append(float(line_comps[2]))
                Y_meas.append(float(line_comps[3])) 
                Z_meas.append(float(line_comps[4])) 
                Roll_meas.append(float(line_comps[4])) 
                Pitch_meas.append(float(line_comps[4])) 
                Yaw_meas.append(float(line_comps[4])) 

    file.close()
    
    if "No_opti" in prefix:
        if j == 8 or j == 9:
            x = X_meas[Idx_no_opti[j]]
            y = Y_meas[Idx_no_opti[j]] - 0.0378
            z = Z_meas[Idx_no_opti[j]]
            roll = Roll_meas[Idx_no_opti[j]]
            pitch = Pitch_meas[Idx_no_opti[j]]
            yaw = Yaw_meas[Idx_no_opti[j]]
        else:
            x = X_meas[Idx_no_opti[j]]
            y = Y_meas[Idx_no_opti[j]]
            z = Z_meas[Idx_no_opti[j]]
            roll = Roll_meas[Idx_no_opti[j]]
            pitch = Pitch_meas[Idx_no_opti[j]]
            yaw = Yaw_meas[Idx_no_opti[j]]
    elif "Opti" in prefix: 
        if j == 0:
            x = X_meas[Idx_opti[j]]
            y = Y_meas[Idx_opti[j]]+0.159
            z = Z_meas[Idx_opti[j]]-0.009
            roll = Roll_meas[Idx_opti[j]]
            pitch = Pitch_meas[Idx_opti[j]]
            yaw = Yaw_meas[Idx_opti[j]]
        else:
            x = X_meas[Idx_opti[j]]
            y = Y_meas[Idx_opti[j]]
            z = Z_meas[Idx_opti[j]]
            roll = Roll_meas[Idx_opti[j]]
            pitch = Pitch_meas[Idx_opti[j]]
            yaw = Yaw_meas[Idx_opti[j]]
    
    rotation_matrix = euler_matrix(roll, pitch, yaw, axes='sxyz')

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
    vec_eef_local[0,0] = perch_length * cos(perche_anlge)
    vec_eef_local[1,0] = perch_length * sin(perche_anlge)
    vec_eef_local[2,0] = z_eef
    vec_eef_local[3,0] = 1.0
    
    eef_global = np.dot(M,vec_eef_local)

    eef_x = eef_global[0]
    eef_y = eef_global[1]
    eef_z = eef_global[2] 
    
    Y.append(eef_y)
    Z.append(eef_z)
###############################################################################################

fig1, ax1 = plt.subplots(1)

ax1.plot([y_target], [z_target], color='k', linestyle = '', marker = '+', markersize=20.0)
for i in range(len(Y)):
    color = 'g'
    if i== 2 or i == 3 or i == 5 or i == 6 or i == 7 or i == 8 or i == 9:
        color = 'r'
    # if i== 3:
    #     color = 'r'
    ax1.plot(Y[i], Z[i]+0.001, color=color, linestyle = '', marker = 'o', markersize=20.0)
ax1.plot(Y[0], Z[0]+0.001, color='g', linestyle = '', marker = 'o', markersize=20.0, label='Perch position')
ax1.set_xlabel('Y (m)')
ax1.set_ylabel('Z (m)')
   

ring_ = patches.Ellipse((y_target, z_target), 2*0.0225, 2*0.0225,color='gray', fill=False, linewidth=48, alpha=0.8)
ax1.add_patch(ring_)
# ring_in = patches.Ellipse((y_target, z_target), 2*0.02, 2*0.02,color='r', fill=False )
# ax1.add_patch(ring_in)
# ring_out = patches.Ellipse((y_target, z_target), 2*0.025, 2*0.025,color='r', fill=False)
# ax1.add_patch(ring_out)


# # Fill the area between the ellipses
# theta = np.linspace(0, 2*np.pi, 200)
# x1 = y_target + 0.02 * np.cos(theta)
# y1 = z_target + 0.025 * np.sin(theta)

# x2 = y_target + 0.02 * np.cos(theta + 0)
# y2 = z_target + 0.025 * np.sin(theta + 0)

# ax1.fill_between(x1, y1, y2, color='gray', alpha=0.5)


ax1.set_xlim([y_target - 0.04, y_target + 0.04])
ax1.set_ylim([z_target - 0.04, z_target + 0.04])

ax1.set_xlabel('Y (m)', fontsize = 40)
ax1.set_ylabel('Z (m)',fontsize = 40)
ax1.tick_params(axis='both', which='major', labelsize=40)  
ax1.set_box_aspect(1)

ax1.legend(fontsize=30)
ax1.grid()
        
plt.show()
    

print('Done')