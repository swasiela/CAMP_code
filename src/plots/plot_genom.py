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

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))

LOG_GENOM = True
ANGLES = False

perch_length_ = 0.5
quad_size = 0.0

# ring_pos_0 = [3.0, 0.0, 1.0, 0.0] # X Y Z Yaw(rad) 
# ring_pos_1 = [5.0, 1.5, 1.5, 0.0] # X Y Z Yaw(rad) 
# ring_pos_2 = [7.0, 3.0, 1.0, 0.0] # X Y Z Yaw(rad) 
# ring_pos_3 = [9.0, 4.5, 1.5, 0.0] # X Y Z Yaw(rad) 

ring_pos_0 = [2.0, 0.0, 1.0, 0.0] # X Y Z Yaw(rad) 
ring_pos_1 = [3.0, 1.0, 1.5, 0.0] # X Y Z Yaw(rad) 
ring_pos_2 = [4.0, 2.0, 1.0, 0.0] # X Y Z Yaw(rad) 
ring_pos_3 = [5.0, 3.0, 1.5, 0.0] # X Y Z Yaw(rad)  


ring_pos_0[0] = ring_pos_0[0] - perch_length_*cos(ring_pos_0[3]) # x = x_eef - dx; dx = h * cos(yaw)
ring_pos_0[1] = ring_pos_0[1] - perch_length_*sin(ring_pos_0[3])

ring_pos_1[0] = ring_pos_1[0] - perch_length_*cos(ring_pos_1[3]) # x = x_eef - dx; dx = h * cos(yaw)
ring_pos_1[1] = ring_pos_1[1] - perch_length_*sin(ring_pos_1[3])

ring_pos_2[0] = ring_pos_2[0] - perch_length_*cos(ring_pos_2[3]) # x = x_eef - dx; dx = h * cos(yaw)
ring_pos_2[1] = ring_pos_2[1] - perch_length_*sin(ring_pos_2[3])

ring_pos_3[0] = ring_pos_3[0] - perch_length_*cos(ring_pos_3[3]) # x = x_eef - dx; dx = h * cos(yaw)
ring_pos_3[1] = ring_pos_3[1] - perch_length_*sin(ring_pos_3[3])

relative_ref_file = os.path.join('src', 'results', 'RAL2024', 'Rings', 'No_opti', 'robust_traj.txt')
relative_file_meas_nom = os.path.join('src', 'results', 'RAL2024', 'Rings', 'No_opti', 'traj_meas_pom_nom.log')
relative_file_meas_perturb = os.path.join('src', 'results', 'RAL2024', 'Rings', 'No_opti', 'traj_meas_pom.log')
relative_file_nom_gaz = os.path.join('src', 'results', 'RAL2024', 'Rings', 'No_opti', 'genom_gazebo_traj_nom.log')
relative_file_perturb_gaz = os.path.join('src', 'results', 'RAL2024', 'Rings', 'No_opti', 'genom_gazebo_traj.log')

file_ref = os.path.join(camp_dir, relative_ref_file)
file_meas_nom = os.path.join(camp_dir, relative_file_meas_nom)
file_meas_perturb = os.path.join(camp_dir, relative_file_meas_perturb)
file_nom_gaz = os.path.join(camp_dir, relative_file_nom_gaz)
file_perturb_gaz = os.path.join(camp_dir, relative_file_perturb_gaz)

Xexec_gaz_nom = []
Yexec_gaz_nom = []
Zexec_gaz_nom = []
Xexec_gaz_perturb = []
Yexec_gaz_perturb = []
Zexec_gaz_perturb = []
QXexec_gaz_nom = []
QYexec_gaz_nom = []
QZexec_gaz_nom = []
QWexec_gaz_nom = []
QXexec_gaz_perturb = []
QYexec_gaz_perturb = []
QZexec_gaz_perturb = []
QWexec_gaz_perturb = []

X = []
Y = []
Z = []
Xnom_simu = []
Ynom_simu = []
Znom_simu = []

X_measure_nom = []
Y_measure_nom = []
Z_measure_nom = []

X_measure_perturb = []
Y_measure_perturb = []
Z_measure_perturb = []

Rx = []
Ry = []
Rz = []
RQW = []
RQX = []
RQY = []
RQZ = []

Index = []

# READING TRAJ FILE
###############################################################################################
file = open(file_ref,"r") 
line = file.readlines()

readingRef = False
readingNom = False
readingUncertainty = False
readingIndex = False

for i in range(len(line)):
    if "Trajectory" in line[i]:
        readingRef = True
        readingNom = False
        readingUncertainty = False
        readingIndex = False
        continue
    if "Uncertainties" in line[i]:
        readingRef = False
        readingNom = False
        readingUncertainty = True
        readingIndex = False
        continue
    if "Index" in line[i]:
        readingRef = False
        readingNom = False
        readingUncertainty = False
        readingIndex = True
        continue
    if "nominal" in line[i]:
        readingRef = False
        readingNom = True
        readingUncertainty = False
        readingIndex = False
        continue
    
    if readingRef:
        line_comps = (line[i].split(";"))
        X.append(float(line_comps[1]))
        Y.append(float(line_comps[2]))
        Z.append(float(line_comps[3]))
    if readingNom:
        line_comps = (line[i].split(";"))
        Xnom_simu.append(float(line_comps[1]))
        Ynom_simu.append(float(line_comps[2]))
        Znom_simu.append(float(line_comps[3]))
        continue
    if readingUncertainty:
        line_comps = (line[i].split(";"))
        Rx.append(float(line_comps[1]) + 0.005)
        Ry.append(float(line_comps[2]) + 0.005)
        Rz.append(float(line_comps[3]) + 0.005)
        if ANGLES:
            RQW.append(float(line_comps[4]))
            RQX.append(float(line_comps[5]))
            RQY.append(float(line_comps[6]))
            RQZ.append(float(line_comps[7]))
    if readingIndex:
        line_comps = (line[i].split(";"))
        Index.append(int(line_comps[1]))

file.close()

# READING TRAJ FILE
###############################################################################################
file = open(file_nom_gaz,"r") 
line = file.readlines()

for i in range(len(line)):
    line_comps = (line[i].split(";"))
    Xexec_gaz_nom.append(float(line_comps[2]))
    Yexec_gaz_nom.append(float(line_comps[3]))
    Zexec_gaz_nom.append(float(line_comps[4]))
    if ANGLES:
        QWexec_gaz_nom.append(float(line_comps[5]))
        QXexec_gaz_nom.append(float(line_comps[6]))
        QYexec_gaz_nom.append(float(line_comps[7]))
        QZexec_gaz_nom.append(float(line_comps[8]))

file.close()

# READING TRAJ FILE
###############################################################################################
file = open(file_perturb_gaz,"r") 
line = file.readlines()

for i in range(len(line)):
    line_comps = (line[i].split(";"))
    Xexec_gaz_perturb.append(float(line_comps[2]))
    Yexec_gaz_perturb.append(float(line_comps[3]))
    Zexec_gaz_perturb.append(float(line_comps[4]))
    if ANGLES:
        QWexec_gaz_perturb.append(float(line_comps[5]))
        QXexec_gaz_perturb.append(float(line_comps[6]))
        QYexec_gaz_perturb.append(float(line_comps[7]))
        QZexec_gaz_perturb.append(float(line_comps[8]))

file.close()

# READING TRAJ FILE
###############################################################################################
file = open(file_meas_perturb,"r") 
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
                        X_measure_perturb.append(number)
                    elif compt_add == 8:   
                        Y_measure_perturb.append(number)
                    elif compt_add == 9:   
                        Z_measure_perturb.append(number)
                    compt_add +=1
                except ValueError:
                    print(f"Ignoring non-numeric value: {elem}")
        else:
            line_comps = (line[i].split(","))
            X_measure_perturb.append(float(line_comps[2]))
            Y_measure_perturb.append(float(line_comps[3])) 
            Z_measure_perturb.append(float(line_comps[4])) 

file.close()
###############################################################################################

# READING TRAJ FILE
###############################################################################################
file = open(file_meas_nom,"r") 
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
                        X_measure_nom.append(number)
                    elif compt_add == 8:   
                        Y_measure_nom.append(number)
                    elif compt_add == 9:   
                        Z_measure_nom.append(number)
                    compt_add +=1
                except ValueError:
                    print(f"Ignoring non-numeric value: {elem}")
        else:
            line_comps = (line[i].split(","))
            X_measure_nom.append(float(line_comps[2]))
            Y_measure_nom.append(float(line_comps[3])) 
            Z_measure_nom.append(float(line_comps[4])) 

file.close()
###############################################################################################

fig0, ax0 = plt.subplots(1) #2D traj XY
fig1, ax1 = plt.subplots(1) #2D traj XZ
fig2, ax2 = plt.subplots(1) #2D traj YZ
fig3, (ax31, ax32, ax33, ax34) = plt.subplots(4) #qw qx qy qz

ax0.plot(X, Y, color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Reference')
ax0.plot(X_measure_nom, Y_measure_nom, color='b', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Measure')
ax0.plot(X_measure_perturb, Y_measure_perturb, color='c', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturbed Measure')
ax0.plot(Xexec_gaz_nom, Yexec_gaz_nom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Gazebo')
ax0.plot(Xexec_gaz_perturb, Yexec_gaz_perturb, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb Gazebo')
ax0.plot(ring_pos_0[0], ring_pos_0[1], color='b', linestyle = '-', marker = 'o', linewidth=2.0, label = 'Ring')
ax0.plot(ring_pos_1[0], ring_pos_1[1], color='b', linestyle = '-', marker = 'o', linewidth=2.0, label = 'Ring')
ax0.plot(ring_pos_2[0], ring_pos_2[1], color='b', linestyle = '-', marker = 'o', linewidth=2.0, label = 'Ring')
ax0.plot(ring_pos_3[0], ring_pos_3[1], color='b', linestyle = '-', marker = 'o', linewidth=2.0, label = 'Ring')
ax0.set_xlabel('X (m)')
ax0.set_ylabel('Y (m)')
ax0.legend()
ax0.grid()

ax1.plot(X, Z, color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Reference')
ax1.plot(X_measure_nom, Z_measure_nom, color='b', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Measure')
ax1.plot(X_measure_perturb, Z_measure_perturb, color='c', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturbed Measure')
ax1.plot(Xexec_gaz_nom, Zexec_gaz_nom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Gazebo')
ax1.plot(Xexec_gaz_perturb, Zexec_gaz_perturb, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb Gazebo')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Z (m)')
ax1.legend()
ax1.grid()

ax2.plot(Y, Z, color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Reference')
ax2.plot(Y_measure_nom, Z_measure_nom, color='b', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Measure')
ax2.plot(Y_measure_perturb, Z_measure_perturb, color='c', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturbed Measure')
ax2.plot(Yexec_gaz_nom, Zexec_gaz_nom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Gazebo')
ax2.plot(Yexec_gaz_perturb, Zexec_gaz_perturb, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb Gazebo')
ax2.set_xlabel('Y (m)')
ax2.set_ylabel('Z (m)')
ax2.legend()
ax2.grid()

if ANGLES:
    ax31.plot(np.linspace(0,len(QWexec_gaz_nom),len(QWexec_gaz_nom)), QWexec_gaz_nom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Gazebo')
    ax31.plot(np.linspace(0,len(QWexec_gaz_perturb),len(QWexec_gaz_perturb)), QWexec_gaz_perturb, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb Gazebo')
    ax31.set_ylabel('qw')
    ax31.legend()
    ax31.grid()
    ax32.plot(np.linspace(0,len(QXexec_gaz_nom),len(QXexec_gaz_nom)), QXexec_gaz_nom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Gazebo')
    ax32.plot(np.linspace(0,len(QXexec_gaz_perturb),len(QXexec_gaz_perturb)), QXexec_gaz_perturb, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb Gazebo')
    ax32.set_ylabel('qx')
    ax32.grid()
    ax33.plot(np.linspace(0,len(QYexec_gaz_nom),len(QYexec_gaz_nom)), QYexec_gaz_nom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Gazebo')
    ax33.plot(np.linspace(0,len(QYexec_gaz_perturb),len(QYexec_gaz_perturb)), QYexec_gaz_perturb, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb Gazebo')
    ax33.set_ylabel('qy')
    ax33.grid()
    ax34.plot(np.linspace(0,len(QZexec_gaz_nom),len(QZexec_gaz_nom)), QZexec_gaz_nom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal Gazebo')
    ax34.plot(np.linspace(0,len(QZexec_gaz_perturb),len(QZexec_gaz_perturb)), QZexec_gaz_perturb, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb Gazebo')
    ax34.set_ylabel('qz')
    ax34.grid()

e_nom_x = []
e_nom_y = []
e_nom_z = []
e_perturb_x = []
e_perturb_y = []
e_perturb_z = []
TubeX = []
TubeY = []
TubeZ = []
for i in range(0,len(Rx)):
    e_x_nom = X[i] - X_measure_nom[i]
    e_y_nom = Y[i] - Y_measure_nom[i]
    e_z_nom = Z[i] - Z_measure_nom[i]
    # e_x_nom = X[i] - Xnom_simu[i]
    # e_y_nom = Y[i] - Ynom_simu[i]
    # e_z_nom = Z[i] - Znom_simu[i]
    e_nom_x.append(e_x_nom)
    e_nom_y.append(e_y_nom)
    e_nom_z.append(e_z_nom)
    e_perturb_x.append(X[i] - X_measure_perturb[i])
    e_perturb_y.append(Y[i] - Y_measure_perturb[i])
    e_perturb_z.append(Z[i] - Z_measure_perturb[i])
    TubeX.append((abs(e_x_nom) + Rx[i] + 0.002 + quad_size))
    TubeY.append((abs(e_y_nom) + Ry[i] + quad_size))
    TubeZ.append((abs(e_z_nom) + Rz[i]))
    
for i in range(0,len(Rx)):
    if i%10==0:
        Uncertainty_XY = patches.Ellipse((X[i], Y[i]), 2*TubeX[i], 2*TubeY[i],color='r', fill=False) # 2*Tube because diamteter is needed 
        ax0.add_patch(Uncertainty_XY)
        Uncertainty_XZ = patches.Ellipse((X[i], Z[i]), 2*TubeX[i], 2*TubeZ[i],color='r', fill=False)
        ax1.add_patch(Uncertainty_XZ)
        Uncertainty_YZ = patches.Ellipse((Y[i], Z[i]), 2*TubeY[i], 2*TubeZ[i],color='r', fill=False)
        ax2.add_patch(Uncertainty_YZ)
        
        if ANGLES:
            Uncertainty_QW = patches.Ellipse((i, QWexec_gaz_nom[i]), 2*RQW[i], 2*0.5 ,color='r', fill=False)
            ax31.add_patch(Uncertainty_QW)
            Uncertainty_QX = patches.Ellipse((i, QXexec_gaz_nom[i]), 2*RQX[i], 2*0.5,color='r', fill=False)
            ax32.add_patch(Uncertainty_QX)
            Uncertainty_QY = patches.Ellipse((i, QYexec_gaz_nom[i]), 2*RQY[i], 2*0.5,color='r', fill=False)
            ax33.add_patch(Uncertainty_QY)
            Uncertainty_QZ = patches.Ellipse((i, QZexec_gaz_nom[i]), 2*RQZ[i], 2*0.5,color='r', fill=False)
            ax34.add_patch(Uncertainty_QZ)
        
        
print(TubeX[Index[1]])
print(TubeY[Index[1]])
print(TubeZ[Index[1]])
plt.show()
    

print('Done')