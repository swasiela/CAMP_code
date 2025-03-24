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

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))

relative_file = os.path.join('src', 'results', 'robust_traj.txt')
file_ref = os.path.join(camp_dir, relative_file)

file_ref = '/home/swasiela/CAMP/src/results/robust_traj.txt'
# file_ref = '/home/swasiela/CAMP/src/results/testNN_working.txt'

X = []
Y = []
Z = []
R = []
P = []
Yaw = []
Vx = []
Vy = []
Vz = []
Wr = []
Wp = []
Wy = []
Ax = []
Ay = []
Az = []
Wrdot = []
Wpdot = []
Wydot = []

XTrueTraj= []
YTrueTraj= []
ZTrueTraj= []
YawTrueTraj= []
VxTrueTraj= []
VyTrueTraj= []
VzTrueTraj= []
WyTrueTraj= []
AxTrueTraj= []
AyTrueTraj= []
AzTrueTraj= []

VxPredTraj= []
VyPredTraj= []
VzPredTraj= []
WyPredTraj= []
AxPredTraj= []
AyPredTraj= []
AzPredTraj= []

Xnom = []
Ynom = []
Znom = []
YawNom = []
RollNom = []
PitchNom = []

Rx_true = []
Ry_true = []
Rz_true = []

u1_true = []
u2_true = []
u3_true = []
u4_true = []

Rx_NN = []
Ry_NN = []
Rz_NN = []

u1_NN = []
u2_NN = []
u3_NN = []
u4_NN = []

time = []

# READING TRAJ FILE
###############################################################################################
file = open(file_ref,"r") 
line = file.readlines()

readingRef = False
readingNom = False
readingTrueTubes = False
readingTrueInputs = False
readingPredicted = False
readingTrueTrajInputs = False
readingPredictedTrajInputs = False
readingTime = False

for i in range(len(line)):
    if "Reference" in line[i]:
        readingRef = True
        readingNom = False
        readingTrueTubes = False
        readingTrueInputs = False
        readingPredicted = False
        readingTrueTrajInputs = False
        readingPredictedTrajInputs = False
        readingTime = False
        continue
    if "True trajectory inputs" in line[i]:
        readingRef = False
        readingNom = False
        readingTrueTubes = False
        readingTrueInputs = False
        readingPredicted = False
        readingTrueTrajInputs = True
        readingPredictedTrajInputs = False
        readingTime = False
        continue
    if "Prediction trajectory inputs" in line[i]:
        readingRef = False
        readingNom = False
        readingTrueTubes = False
        readingTrueInputs = False
        readingPredicted = False
        readingTrueTrajInputs = False
        readingPredictedTrajInputs = True
        readingTime = False
        continue
    if "True tubes" in line[i]:
        readingRef = False
        readingNom = False
        readingTrueTubes = True
        readingTrueInputs = False
        readingPredicted = False
        readingTrueTrajInputs = False
        readingPredictedTrajInputs = False
        readingTime = False
        continue
    if "True inputs" in line[i]:
        readingRef = False
        readingNom = False
        readingTrueTubes = False
        readingTrueInputs = True
        readingPredicted = False
        readingTrueTrajInputs = False
        readingPredictedTrajInputs = False
        readingTime = False
        continue
    if "Predicted tubes" in line[i]:
        readingRef = False
        readingNom = False
        readingTrueTubes = False
        readingTrueInputs = False
        readingPredicted = True
        readingTrueTrajInputs = False
        readingPredictedTrajInputs = False
        readingTime = False
        continue
    if "Nominal" in line[i]:
        readingRef = False
        readingNom = True
        readingTrueTubes = False
        readingTrueInputs = False
        readingPredicted = False
        readingTrueTrajInputs = False
        readingPredictedTrajInputs = False
        readingTime = False
        continue
    if "Time" in line[i]:
        readingRef = False
        readingNom = False
        readingTrueTubes = False
        readingTrueInputs = False
        readingPredicted = False
        readingTrueTrajInputs = False
        readingPredictedTrajInputs = False
        readingTime = True
        continue
    
    if readingRef:
        line_comps = (line[i].split(";"))
        X.append(float(line_comps[1]))
        Y.append(float(line_comps[2]))
        Z.append(float(line_comps[3]))
        Yaw.append(float(line_comps[4]))
        Vx.append(float(line_comps[5]))
        Vy.append(float(line_comps[6]))
        Vz.append(float(line_comps[7]))
        Wy.append(float(line_comps[8]))
        Ax.append(float(line_comps[9]))
        Ay.append(float(line_comps[10]))
        Az.append(float(line_comps[11]))
    if readingNom:
        line_comps = (line[i].split(";"))
        Xnom.append(float(line_comps[1]))
        Ynom.append(float(line_comps[2]))
        Znom.append(float(line_comps[3]))
        
        qw = float(line_comps[4])
        qx = float(line_comps[5])
        qy = float(line_comps[6])
        qz = float(line_comps[7])

        # Compute roll (phi)
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))

        # Compute pitch (theta)
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))  # Clamp to handle numerical issues

        # Compute yaw (psi)
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
        
        RollNom.append(roll)
        PitchNom.append(pitch)
        YawNom.append(yaw)
            
    if readingTrueTubes:
        line_comps = (line[i].split(";"))
        Rx_true.append(float(line_comps[1]))
        Ry_true.append(float(line_comps[2]))
        Rz_true.append(float(line_comps[3]))
    if readingTrueTrajInputs:
        line_comps = (line[i].split(";"))
        XTrueTraj.append(float(line_comps[1]))
        YTrueTraj.append(float(line_comps[2]))
        ZTrueTraj.append(float(line_comps[3]))
        YawTrueTraj.append(float(line_comps[4]))
        VxTrueTraj.append(float(line_comps[5]))
        VyTrueTraj.append(float(line_comps[6]))
        VzTrueTraj.append(float(line_comps[7]))
        WyTrueTraj.append(float(line_comps[8]))
        AxTrueTraj.append(float(line_comps[9]))
        AyTrueTraj.append(float(line_comps[10]))
        AzTrueTraj.append(float(line_comps[11]))
    if readingPredictedTrajInputs:
        line_comps = (line[i].split(";"))
        VxPredTraj.append(float(line_comps[1]))
        VyPredTraj.append(float(line_comps[2]))
        VzPredTraj.append(float(line_comps[3]))
        WyPredTraj.append(float(line_comps[4]))
        AxPredTraj.append(float(line_comps[5]))
        AyPredTraj.append(float(line_comps[6]))
        AzPredTraj.append(float(line_comps[7]))
    if readingTrueInputs:
        line_comps = (line[i].split(";"))
        u1_true.append(float(line_comps[1]))
        u2_true.append(float(line_comps[2]))
        u3_true.append(float(line_comps[3]))
        u4_true.append(float(line_comps[4]))
    if readingPredicted:
        line_comps = (line[i].split(";"))
        Rx_NN.append(float(line_comps[1]))
        Ry_NN.append(float(line_comps[2]))
        Rz_NN.append(float(line_comps[3]))
        u1_NN.append(float(line_comps[4]))
        u2_NN.append(float(line_comps[5]))
        u3_NN.append(float(line_comps[6]))
        u4_NN.append(float(line_comps[7]))
    if readingTime:
        line_comps = (line[i].split(";"))
        time.append(float(line_comps[1]))

file.close()
###########################################################################################################""

fig0, ax0 = plt.subplots(1) #2D traj XY
fig1, ax1 = plt.subplots(1) #2D traj XZ
fig2, ax2 = plt.subplots(1) #2D traj YZ

fig3, (ax31, ax32, ax33, ax34) = plt.subplots(4)
fig4, (ax41, ax42, ax43) = plt.subplots(3)

fig5, ax5 = plt.subplots(1) #2D traj XY
fig6, ax6 = plt.subplots(1) #2D traj XZ
fig7, ax7 = plt.subplots(1) #2D traj YZ
fig9, (ax91, ax92, ax93) = plt.subplots(3)

fig8, (ax81, ax82, ax83, ax84) = plt.subplots(4) #u

ax0.plot(np.linspace(0,len(Rx_true),len(Rx_true)), Rx_true, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax0.plot(np.linspace(0,len(Rx_NN),len(Rx_NN)), Rx_NN, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax0.set_xlabel('Step')
ax0.set_ylabel('Rx (m)')
ax0.legend()
ax0.grid()

ax1.plot(np.linspace(0,len(Ry_true),len(Ry_true)), Ry_true, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax1.plot(np.linspace(0,len(Ry_NN),len(Ry_NN)), Ry_NN, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax1.set_xlabel('Setp ')
ax1.set_ylabel('Ry (m)')
ax1.legend()
ax1.grid()

ax2.plot(np.linspace(0,len(Rz_true),len(Rz_true)), Rz_true, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax2.plot(np.linspace(0,len(Rz_NN),len(Rz_NN)), Rz_NN, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax2.set_xlabel('Step')
ax2.set_ylabel('Rz (m)')
ax2.legend()
ax2.grid()

ax31.plot(np.linspace(0,len(VxTrueTraj),len(VxTrueTraj)), VxTrueTraj, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax31.plot(np.linspace(0,len(VxPredTraj),len(VxPredTraj)), VxPredTraj, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax31.set_ylabel('Vx')
ax31.grid()
ax32.plot(np.linspace(0,len(VyTrueTraj),len(VyTrueTraj)), VyTrueTraj, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax32.plot(np.linspace(0,len(VyPredTraj),len(VyPredTraj)), VyPredTraj, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax32.set_ylabel('Vy')
ax32.grid()
ax33.plot(np.linspace(0,len(VzTrueTraj),len(VzTrueTraj)), VzTrueTraj, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax33.plot(np.linspace(0,len(VzPredTraj),len(VzPredTraj)), VzPredTraj, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax33.set_ylabel('Vz')
ax33.grid()
ax34.plot(np.linspace(0,len(WyTrueTraj),len(WyTrueTraj)), WyTrueTraj, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax34.plot(np.linspace(0,len(WyPredTraj),len(WyPredTraj)), WyPredTraj, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax34.set_ylabel('Wyaw')
ax34.grid()
ax34.set_xlabel('Step')

ax41.plot(np.linspace(0,len(AxTrueTraj),len(AxTrueTraj)), AxTrueTraj, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax41.plot(np.linspace(0,len(AxPredTraj),len(AxPredTraj)), AxPredTraj, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax41.set_ylabel('Ax')
ax41.grid()
ax42.plot(np.linspace(0,len(AyTrueTraj),len(AyTrueTraj)), AyTrueTraj, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax42.plot(np.linspace(0,len(AyPredTraj),len(AyPredTraj)), AyPredTraj, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax42.set_ylabel('Ay')
ax42.grid()
ax43.plot(np.linspace(0,len(AzTrueTraj),len(AzTrueTraj)), AzTrueTraj, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax43.plot(np.linspace(0,len(AzPredTraj),len(AzPredTraj)), AzPredTraj, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax43.set_ylabel('Az')
ax43.grid()
ax43.set_xlabel('Step')

ax5.plot(time, X, color='k', linestyle = '-', marker = '+', linewidth=2.0, label = 'Reference')
ax5.plot(time, Xnom, color='y', linestyle = '-', marker = 'o', linewidth=2.0, label = 'Nominal')
ax5.set_xlabel('Step')
ax5.set_ylabel('X (m)')
ax5.legend()
ax5.grid()

ax6.plot(time, Y, color='k', linestyle = '-', marker = '+', linewidth=2.0, label = 'Reference')
ax6.plot(time, Ynom, color='y', linestyle = '-', marker = 'o', linewidth=2.0, label = 'Nominal')
ax6.set_xlabel('Step')
ax6.set_ylabel('Y (m)')
ax6.legend()
ax6.grid()

ax7.plot(time, Z, color='k', linestyle = '-', marker = '+', linewidth=2.0, label = 'Reference')
ax7.plot(time, Znom, color='y', linestyle = '-', marker = 'o', linewidth=2.0, label = 'Nominal')
ax7.set_xlabel('Step')
ax7.set_ylabel('Z (m)')
ax7.legend()
ax7.grid()

ax81.plot(np.linspace(0,len(u1_true),len(u1_true)), u1_true, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax81.plot(np.linspace(0,len(u1_NN),len(u1_NN)), u1_NN, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax81.set_ylabel('U1')
ax81.grid()
ax82.plot(np.linspace(0,len(u2_true),len(u2_true)), u2_true, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax82.plot(np.linspace(0,len(u2_NN),len(u2_NN)), u2_NN, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax82.set_ylabel('U2')
ax82.grid()
ax83.plot(np.linspace(0,len(u3_true),len(u3_true)), u3_true, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax83.plot(np.linspace(0,len(u3_NN),len(u3_NN)), u3_NN, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax83.set_ylabel('U3')
ax83.grid()
ax84.plot(np.linspace(0,len(u4_true),len(u4_true)), u4_true, color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'True')
ax84.plot(np.linspace(0,len(u4_NN),len(u4_NN)), u4_NN, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'NN')
ax84.set_ylabel('U4')
ax84.grid()
ax84.set_xlabel('Step')


ax91.plot(time, RollNom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal')
ax91.set_ylabel('Roll')
ax91.grid()
ax92.plot(time, PitchNom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal')
ax92.set_ylabel('Pitch')
ax92.grid()
ax93.plot(time, Yaw, color='k', linestyle = '-', marker = '+', linewidth=2.0, label = 'Reference')
ax93.plot(time, YawNom, color='y', linestyle = '-', marker = 'o', linewidth=2.0, label = 'Nominal')
ax93.set_ylabel('Yaw')
ax93.grid()
ax93.set_xlabel('Step')
    
# for i in range(0,len(Rx_true)):
#     if i%30==0:
#         Uncertainty_XY = patches.Ellipse((Xnom[i], Ynom[i]), 2*Rx_true[i], 2*Ry_true[i],color='g', fill=False) # 2*Tube because diamteter is needed 
#         ax0.add_patch(Uncertainty_XY)
#         Uncertainty_XZ = patches.Ellipse((Xnom[i], Znom[i]), 2*Rx_true[i], 2*Rz_true[i],color='g', fill=False)
#         ax1.add_patch(Uncertainty_XZ)
#         Uncertainty_YZ = patches.Ellipse((Ynom[i], Znom[i]), 2*Ry_true[i], 2*Rz_true[i],color='g', fill=False)
#         ax2.add_patch(Uncertainty_YZ)
        
#         Uncertainty_XYNN = patches.Ellipse((Xnom[i], Ynom[i]), 2*Rx_NN[i], 2*Ry_NN[i],color='r', fill=False) # 2*Tube because diamteter is needed 
#         ax0.add_patch(Uncertainty_XYNN)
#         Uncertainty_XZNN = patches.Ellipse((Xnom[i], Znom[i]), 2*Rx_NN[i], 2*Rz_NN[i],color='r', fill=False)
#         ax1.add_patch(Uncertainty_XZNN)
#         Uncertainty_YZNN = patches.Ellipse((Ynom[i], Znom[i]), 2*Ry_NN[i], 2*Rz_NN[i],color='r', fill=False)
#         ax2.add_patch(Uncertainty_YZNN)

plt.show()
    

print('Done')