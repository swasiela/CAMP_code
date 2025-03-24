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

from math import *
import numpy as np
import pybullet as p

LOG_GENOM = True

xstart = -2.0
ystart = -1.0
zstart = 1.5

nb_col = 0

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))

relative_file = os.path.join('src', 'results', 'RAL2024', 'window', 'RRTstar', 'Exp_RRTstar', 'Traj_')
prefix = os.path.join(camp_dir, relative_file)

def euler_to_quaternion(roll, pitch, yaw):
    # Calculate half angles
    roll /= 2.0
    pitch /= 2.0
    yaw /= 2.0
    
    # Calculate sin and cos values
    sr = np.sin(roll)
    cr = np.cos(roll)
    sp = np.sin(pitch)
    cp = np.cos(pitch)
    sy = np.sin(yaw)
    cy = np.cos(yaw)
    
    # Calculate quaternion components
    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy
    
    return q_w, q_x, q_y, q_z

def quaternion_to_euler(w, x, y, z):
    # Calculate pitch (theta)
    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1.0:
        pitch = np.pi / 2.0 if sin_pitch > 0 else -np.pi / 2.0
    else:
        pitch = np.arcsin(sin_pitch)
    
    # Calculate roll (phi)
    cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    sin_roll = 2.0 * (w * x + y * z)
    roll = np.arctan2(sin_roll, cos_pitch)
    
    # Calculate yaw (psi)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (z * z + y * y)
    yaw = np.arctan2(sin_yaw, cos_yaw)
    
    return roll, pitch, yaw

X_measure_real = []
Y_measure_real = []
Z_measure_real = []
R_measure_real = []
P_measure_real = []
Yaw_measure_real = []

# READING TRAJ FILE
###############################################################################################
for i in range(0,1):
    X_meas = []
    Y_meas = []
    Z_meas = []
    R_meas = []
    P_meas = []
    Yaw_meas = []
    fl_name = prefix+str(i)+"/pom.log"
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
                            R_meas.append(number)
                        elif compt_add == 11:
                            P_meas.append(number)
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
                R_meas.append(float(line_comps[5]))
                P_meas.append(float(line_comps[6])) 
                Yaw_meas.append(float(line_comps[7])) 

    file.close()
    Xsim_copy = X_meas
    Ysim_copy = Y_meas
    Zsim_copy = Z_meas
    Rsim_copy = R_meas
    Psim_copy = P_meas
    Yawsim_copy = Yaw_meas
    X_measure_real.append(Xsim_copy)
    Y_measure_real.append(Ysim_copy)
    Z_measure_real.append(Zsim_copy)
    R_measure_real.append(Rsim_copy)
    P_measure_real.append(Psim_copy)
    Yaw_measure_real.append(Yawsim_copy)
###############################################################################################

# Setup PyBullet
if p.isConnected():
    p.disconnect()
p_ = p.connect(p.GUI) 
# Load the robot URDF

relative_urdf = os.path.join('src', 'robots', 'urdf', 'quadrotor.urdf')
robot_urdf_path = os.path.join(camp_dir, relative_urdf)
robot_id = p.loadURDF(robot_urdf_path, [xstart, ystart, zstart])  # Provide the initial pose

red_color = [1, 0, 0, 1]  # Red color with full opacity
p.changeVisualShape(robot_id, 0, rgbaColor=red_color)

# Load the OBJ environment
env = []
relative_env = os.path.join('src', 'robots', 'urdf', 'scene.urdf')
obj_environment_path_1 = os.path.join(camp_dir, relative_env)
env.append(p.loadURDF(obj_environment_path_1, [0, 0, 0]))
p.stepSimulation()

for i in range(len(X_measure_real)):
    has_collision = False
    
    for j in range(len(X_measure_real[i])):
        # Check collision
        qw, qx, qy, qz = euler_to_quaternion(R_measure_real[i][j], P_measure_real[i][j], Yaw_measure_real[i][j])
        p.resetBasePositionAndOrientation(robot_id, [X_measure_real[i][j], Y_measure_real[i][j], Z_measure_real[i][j]], [qx, qy, qz, qw]) #NED frame

        for elem in env:
            p.resetBasePositionAndOrientation(elem, [0,0,0], [0,0,0,1])
        
        p.stepSimulation()
        for elem in env:
            # Perform collision checking
            collisions = p.getContactPoints(robot_id, elem)
            if collisions:
                dist = p.getClosestPoints(robot_id, elem, distance=0.1) #PyBullet use a margin of 0.04m for collisions
                if dist[0][8] < 0:
                    has_collision = True
                    nb_col += 1
                    break
        if has_collision:
            break

print('Nb collision = ', nb_col)