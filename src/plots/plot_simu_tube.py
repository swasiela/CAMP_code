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
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math

from scipy.spatial.transform import Rotation as R

assert sys.argv[1] == "Unicycle" or sys.argv[1] == "Quadrotor", "Error: Argument must be either 'Unicycle' or 'Quadrotor'."

ROBOT = sys.argv[1] # Unicycle Quadrotor

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))

relative_file_ref = os.path.join('src', 'results','robust_traj.txt')
relative_file_exec_simu = os.path.join('src', 'results', 'simulated_traj.txt')

# relative_file_ref = os.path.join('src', 'results', 'RAL2024', 'window', 'R_SARRTstar_NN', 'Traj_10','robust_traj.txt')
# relative_file_exec_simu = os.path.join('src', 'results', 'RAL2024', 'window', 'R_SARRTstar_NN', 'Traj_10', 'simulated_traj.txt')

file_ref = os.path.join(camp_dir, relative_file_ref)
file_exec_simu = os.path.join(camp_dir, relative_file_exec_simu)

e_x = []
e_y = []
e_z = []

e_r_x = []
e_r_y = []
e_r_z = []

X = []
Y = []
Z = []
Xnom = []
Ynom = []
Znom = []
Xsimu = []
Ysimu = []
Zsimu = []
Yawsimu = []
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

Rx = []
Ry = []
Rz = []

Index = []
Validity = []

# READING TRAJ FILE
###############################################################################################
file = open(file_ref,"r") 
line = file.readlines()

readingRef = False
readingNom = False
readingUncertainty = False
readingIndex = False
readingInputs = False

for i in range(len(line)):
    if "Reference" in line[i]:
        readingRef = True
        readingNom = False
        readingUncertainty = False
        readingIndex = False
        readingInputs = False
        continue
    if "Uncertainties" in line[i] or "tubes" in line[i]:
        readingRef = False
        readingNom = False
        readingUncertainty = True
        readingIndex = False
        readingInputs = False
        continue
    if "inputs" in line[i]:
        readingRef = False
        readingNom = False
        readingUncertainty = False
        readingIndex = False
        readingInputs = True
        continue
    if "Index" in line[i]:
        readingRef = False
        readingNom = False
        readingUncertainty = False
        readingIndex = True
        readingInputs = False
        continue
    if "Nominal" in line[i]:
        readingRef = False
        readingNom = True
        readingUncertainty = False
        readingIndex = False
        readingInputs = False
        continue
    
    if readingRef:
        if ROBOT == "Unicycle":
            line_comps = (line[i].split(";"))
            if(float(line_comps[4]) == 0.0):
                continue
            if(float(line_comps[5]) == 0.0):
                continue
            Vx.append(float(line_comps[4]))
            Vy.append(float(line_comps[5]))
    if readingNom:
        continue
    if readingUncertainty:
        line_comps = (line[i].split(";"))
        Rx.append(float(line_comps[1]))
        Ry.append(float(line_comps[2]))
        if ROBOT == "Quadrotor":
            Rz.append(float(line_comps[3]))
    if readingIndex:
        continue

file.close()

# READING TRAJ FILE
###############################################################################################
file = open(file_exec_simu,"r") 
line = file.readlines()

readingRef = False
readingNom = False
readingSimu = False
readingIndex = False
readingValidity = False

Xsim = []
Ysim = []
Zsim = []
compt_simu = -1

for i in range(len(line)):
    if "Trajectory" in line[i]:
        readingRef = False
        readingNom = False
        readingSimu = True
        readingIndex = False
        readingValidity = False
        if compt_simu == -1:
            compt_simu += 1
            continue
        Xsim_copy = Xsim
        Ysim_copy = Ysim
        Zsim_copy = Zsim
        Xsimu.append(Xsim_copy)
        Ysimu.append(Ysim_copy)
        Zsimu.append(Zsim_copy)
        Xsim = []
        Ysim = []
        Zsim = []
        compt_simu += 1
        continue
    if "Reference" in line[i]:
        readingRef = True
        readingNom = False
        readingSimu = False
        readingIndex = False
        readingValidity = False
        continue
    if "Validity" in line[i]:
        readingRef = False
        readingNom = False
        readingSimu = False
        readingIndex = False
        readingValidity = True
        line_comps = (line[i].split(";"))
        Validity.append(float(line_comps[1]))
        continue
    if "Index" in line[i]:
        readingRef = False
        readingNom = False
        readingSimu = False
        readingIndex = True
        readingValidity = False
        continue
    if "Nominal" in line[i]:
        readingRef = False
        readingNom = True
        readingSimu = False
        readingIndex = False
        readingValidity = False
        continue
    
    if "TimeVec" in line[i]:
        readingRef = False
        readingNom = False
        readingSimu = False
        readingIndex = False
        readingValidity = False
        continue
    
    if readingRef:
        line_comps = (line[i].split(";"))
        X.append(float(line_comps[1]))
        Y.append(float(line_comps[2]))
        if ROBOT == "Quadrotor":
            Z.append(float(line_comps[3]))
            Yaw.append(float(line_comps[4]))
            Vx.append(float(line_comps[5]))
            Vy.append(float(line_comps[6]))
            Vz.append(float(line_comps[7]))
            Wy.append(float(line_comps[8]))
            Ax.append(float(line_comps[9]))
            Ay.append(float(line_comps[10]))
            Az.append(float(line_comps[11]))
        if ROBOT == "Unicycle":
            Yaw.append(float(line_comps[3]))
    if readingNom:
        line_comps = (line[i].split(";"))
        Xnom.append(float(line_comps[1]))
        Ynom.append(float(line_comps[2]))
        Znom.append(float(line_comps[3]))
    if readingSimu:
        line_comps = (line[i].split(";"))
        Xsim.append(float(line_comps[1]))
        Ysim.append(float(line_comps[2]))
        Zsim.append(float(line_comps[3]))
    if readingIndex:
        line_comps = (line[i].split(";"))
        Index.append(float(line_comps[1]))

Xsim_copy = Xsim
Ysim_copy = Ysim
Zsim_copy = Zsim
Xsimu.append(Xsim_copy)
Ysimu.append(Ysim_copy)
Zsimu.append(Zsim_copy)
file.close()
###############################################################################################

fig0, ax0 = plt.subplots(1) #2D traj XY
fig1, ax1 = plt.subplots(1) #Yaw
# fig2, ax2 = plt.subplots(1) #2D traj YZ
fig3, ax3 = plt.subplots(1) #X
fig4, ax4 = plt.subplots(1) #Y
fig5, ax5 = plt.subplots(1) # Vx
fig6, ax6 = plt.subplots(1) # Vy

ax0.plot(X, Y, color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Reference')
ax0.plot(Xnom, Ynom, color='y', linestyle = '', marker = '+', linewidth=2.0, label = 'Nominal')
for i in range(0,len(Xsimu)):
    ax0.plot(Xsimu[i], Ysimu[i], color='g', linestyle = '', marker = 'o', linewidth=2.0)
# ax0.plot(Xsimu[0], Ysimu[0], color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb')
ax0.set_xlabel('X (m)')
ax0.set_ylabel('Y (m)')
ax0.legend()
ax0.grid()

ax1.plot(np.linspace(0,len(Znom),len(Znom)), Znom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal')
for i in range(0,len(Zsimu)):
    ax1.plot(np.linspace(0,len(Zsimu[i]),len(Zsimu[i])), Zsimu[i], color='g', linestyle = '-', marker = '', linewidth=2.0)
ax1.set_xlabel('T (s)')
ax1.set_ylabel('Yaw (rad)')
ax1.legend()
ax1.grid()

# ax1.plot(X, Z, color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Reference')
# ax1.plot(Xnom, Znom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal')
# for i in range(0,len(Xsimu)):
#     ax1.plot(Xsimu[i], Zsimu[i], color='g', linestyle = '-', marker = '', linewidth=2.0)
# # ax1.plot(Xsimu[0], Zsimu[0], color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb')
# ax1.set_xlabel('X (m)')
# ax1.set_ylabel('Z (m)')
# ax1.legend()
# ax1.grid()

# ax2.plot(Y, Z, color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Reference')
# ax2.plot(Ynom, Znom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal')
# for i in range(0,len(Xsimu)):
#     ax2.plot(Ysimu[i], Zsimu[i], color='g', linestyle = '-', marker = '', linewidth=2.0)
# # ax2.plot(Ysimu[0], Zsimu[0], color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Perturb')
# ax2.set_xlabel('Y (m)')
# ax2.set_ylabel('Z (m)')
# ax2.legend()
# ax2.grid()

ax3.plot(np.linspace(0,len(X),len(X)), X, color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Reference')
ax3.plot(np.linspace(0,len(Rx),len(Rx)), [sum(x) for x in zip(Xnom, Rx)], color='r', linestyle = '-', marker = '', linewidth=2.0, label = 'Tube')
ax3.plot(np.linspace(0,len(Rx),len(Rx)), np.subtract(Xnom, Rx), color='r', linestyle = '-', marker = '', linewidth=2.0 )
ax3.plot(np.linspace(0,len(Xnom),len(Xnom)), Xnom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal')
for i in range(0,len(Xsimu)):
    ax3.plot(np.linspace(0,len(Xsimu[i]),len(Xsimu[i])), Xsimu[i], color='g', linestyle = '-', marker = '', linewidth=2.0)
ax3.set_xlabel('T (s)')
ax3.set_ylabel('X (m)')
ax3.legend()
ax3.grid()

ax4.plot(np.linspace(0,len(Y),len(Y)), Y, color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Reference')
ax4.plot(np.linspace(0,len(Ry),len(Ry)), [sum(x) for x in zip(Ynom, Ry)], color='r', linestyle = '-', marker = '', linewidth=2.0, label = 'Tube')
ax4.plot(np.linspace(0,len(Ry),len(Ry)), np.subtract(Ynom, Ry), color='r', linestyle = '-', marker = '', linewidth=2.0 )
ax4.plot(np.linspace(0,len(Ynom),len(Ynom)), Ynom, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Nominal')
for i in range(0,len(Ysimu)):
    ax4.plot(np.linspace(0,len(Ysimu[i]),len(Ysimu[i])), Ysimu[i], color='g', linestyle = '-', marker = '', linewidth=2.0)
ax4.set_xlabel('T (s)')
ax4.set_ylabel('Y (m)')
ax4.legend()
ax4.grid()

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
    # e_x_nom = X[i] - Xnom[i]
    # e_y_nom = Y[i] - Ynom[i]
    # e_z_nom = Z[i] - Znom[i]
    e_x_nom = 0
    e_y_nom = 0
    e_z_nom = 0
    e_nom_x.append(e_x_nom)
    e_nom_y.append(e_y_nom)
    e_nom_z.append(e_z_nom)
    TubeX.append((abs(e_x_nom) + Rx[i]))
    TubeY.append((abs(e_y_nom) + Ry[i]))
    if ROBOT == "Quadrotor":
        TubeZ.append((abs(e_z_nom) + Rz[i]))
    
for i in range(0,len(Rx)):
    if i%1==0:
        # Uncertainty_XY = patches.Ellipse((X[i], Y[i]), 2*TubeX[i], 2*TubeY[i],color='r', fill=False) # 2*Tube because diamteter is needed 
        # ax0.add_patch(Uncertainty_XY)
        # Uncertainty_XZ = patches.Ellipse((X[i], Z[i]), 2*TubeX[i], 2*TubeZ[i],color='r', fill=False)
        # ax1.add_patch(Uncertainty_XZ)
        # Uncertainty_YZ = patches.Ellipse((Y[i], Z[i]), 2*TubeY[i], 2*TubeZ[i],color='r', fill=False)
        # ax2.add_patch(Uncertainty_YZ)
        Uncertainty_XY_ell = patches.Ellipse((Xnom[i], Ynom[i]), 2*TubeX[i], 2*TubeY[i],color='y', fill=False) # 2*Tube because diamteter is needed 
        ax0.add_patch(Uncertainty_XY_ell)
        Uncertainty_XY_rec = patches.Rectangle((Xnom[i]-TubeX[i], Ynom[i]-TubeY[i]), 2*TubeX[i], 2*TubeY[i],color='r', fill=False) # 2*Tube because diamteter is needed 
        ax0.add_patch(Uncertainty_XY_rec)
        # Uncertainty_XZ = patches.Ellipse((Xnom[i], Znom[i]), 2*TubeX[i], 2*TubeZ[i],color='r', fill=False)
        # ax1.add_patch(Uncertainty_XZ)
        # Uncertainty_YZ = patches.Ellipse((Ynom[i], Znom[i]), 2*TubeY[i], 2*TubeZ[i],color='r', fill=False)
        # ax2.add_patch(Uncertainty_YZ)

# V = np.empty((0,len(Vx)))
# V = np.vstack((V, Vx))
# V = np.vstack((V, Vy))
# V_norm = np.linalg.norm(V, axis=0)

V_norm = [math.sqrt(x**2 + y**2) for x, y in zip(Vx[:-1], Vy[:-1])]

# Rad = np.empty((0,len(TubeX)))
# Rad = np.vstack((Rad, TubeX))
# Rad = np.vstack((Rad, TubeY))
# Rad_norm = np.linalg.norm(Rad, axis=0)

# Jx = np.diff(Ax)

# ax5.plot(np.linspace(0,len(Vx[:-1]),len(Vx[:-1])), Vx[:-1], color='g', linestyle = '-', marker = '', linewidth=2.0, label = 'Vx')
# ax5.plot(np.linspace(0,len(Ax[:-1]),len(Ax[:-1])), Ax[:-1], color='b', linestyle = '-', marker = '', linewidth=2.0, label = 'Ax')
# ax5.plot(np.linspace(0,len(Jx[:-1]),len(Jx[:-1])), Jx[:-1], color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Jx')
# ax5.plot(np.linspace(0,len(TubeX),len(TubeX)), TubeX, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Rx')
# ax5.set_xlabel('Step')
# ax5.set_ylabel('Vx')
# ax5.legend()
# ax5.grid()

# ax6.plot(np.linspace(0,len(Vy),len(Vy)), Vy, color='k', linestyle = '-', marker = '', linewidth=2.0, label = 'Vy')
# ax6.plot(np.linspace(0,len(TubeY),len(TubeY)), TubeY, color='y', linestyle = '-', marker = '', linewidth=2.0, label = 'Ry')
# ax6.set_xlabel('Step')
# ax6.set_ylabel('Vy')
# ax6.legend()
# ax6.grid()

print('Max R: ', max(np.max(TubeX),np.max(TubeY)))
plt.show()
    
print('Done')