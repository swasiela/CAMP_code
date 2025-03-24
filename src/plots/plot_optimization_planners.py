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

import os
from os import listdir
from os.path import isfile, join
from scipy.interpolate import interp1d
import scipy.optimize

import subprocess
import sys
import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from math import *

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))

# INITIALIZATION
###############################################################################################

relative_folder_shct = os.path.join('src', 'results', 'LocalOpt', 'Methods', 'Shortcut')
relative_folder_kinoshct = os.path.join('src', 'results', 'LocalOpt', 'Methods', 'KinoShortcut', 'FixedSampling')
relative_folder_nlopt = os.path.join('src', 'results', 'LocalOpt', 'Methods', 'Nlopt')

folder_shct = os.path.join(camp_dir, relative_folder_shct)
folder_nlopt = os.path.join(camp_dir, relative_folder_nlopt)
folder_kinoshct = os.path.join(camp_dir, relative_folder_kinoshct)

# For the initial guess
Xinit = []
Yinit = []
Zinit = []
Yawinit = []

Vxinit = []
Vyinit = []
Vzinit = []
Wyinit = []

Axinit = []
Ayinit = []
Azinit = []
Wydotinit = []

Cost_Init = np.array([])

# For the Shortcut
Xshct = []
Yshct = []
Zshct = []
Yawshct = []

Vxshct = []
Vyshct = []
Vzshct = []
Wyshct = []

Axshct = []
Ayshct = []
Azshct = []
Wydotshct = []

Cost_Shct = np.array([])
Time_Shct = np.array([])

# For the KinoShortcut fixed radius
Xkinoshct_fixed_combi = []
Ykinoshct_fixed_combi = []
Zkinoshct_fixed_combi = []
Yawkinoshct_fixed_combi = []

Vxkinoshct_fixed_combi = []
Vykinoshct_fixed_combi = []
Vzkinoshct_fixed_combi = []
Wykinoshct_fixed_combi = []

Axkinoshct_fixed_combi = []
Aykinoshct_fixed_combi = []
Azkinoshct_fixed_combi = []
Wydotkinoshct_fixed_combi = []

Cost_KinoShct_fixed_combi = np.array([])
Time_KinoShct_fixed_combi = np.array([])

# For Nlopt method
Xnlopt = []
Ynlopt = []
Znlopt = []
Yawnlopt = []
Vxnlopt = []
Vynlopt = []
Vznlopt = []
Wynlopt = []
Axnlopt = []
Aynlopt = []
Aznlopt = []
Wydotnlopt = []
Cost_Nlopt = []
Time_Nlopt = []

# For STOMP method
Xstomp = []
Ystomp = []
Zstomp = []
Yawstomp = []
Vxstomp = []
Vystomp = []
Vzstomp = []
Wystomp = []
Axstomp = []
Aystomp = []
Azstomp = []
Wydotstomp = []
Cost_stomp = []
Time_stomp = []


# READING RANDOM SHORTCUT
###############################################################################################
subfiles = [f for f in os.listdir(folder_shct) if os.path.isfile(os.path.join(folder_shct, f))]
for x in subfiles:
    file = open(os.path.join(folder_shct, x),"r") 
    line = file.readlines()
    
    readingTraj = False
    readingCost = False

    X = []
    Y = []
    Z = []
    Yaw = []

    Vx = []
    Vy = []
    Vz = []
    Wy = []

    Ax = []
    Ay = []
    Az = []
    Wydot = []

    Cost = np.array([])
    
    compt_traj = 0

    for i in range(len(line)):
        if "Trajectory" in line[i]:
            readingTraj = True
            readingCost = False
            if compt_traj == 1:
                Xinit.append(X.copy())
                Yinit.append(Y.copy())
                Zinit.append(Z.copy())
                Yawinit.append(Yaw.copy())
                Vxinit.append(Vx.copy())
                Vyinit.append(Vy.copy())
                Vzinit.append(Vz.copy())
                Wyinit.append(Wy.copy())
                Axinit.append(Ax.copy())
                Ayinit.append(Ay.copy())
                Azinit.append(Az.copy())
                Wydotinit.append(Wydot.copy())
                Cost_Init = np.append(Cost_Init, Cost.copy())
            X.clear()
            X.clear()
            Y.clear()
            Z.clear()
            Yaw.clear()
            Vx.clear()
            Vy.clear()
            Vz.clear()
            Wy.clear()
            Ax.clear()
            Ay.clear()
            Az.clear()
            Wydot.clear()
            Cost = np.array([])
            compt_traj += 1
            continue
        if "Cost" in line[i]:
            readingTraj = False
            readingCost = True
            continue
        if "Time" in line[i]:
            readingTraj = False
            readingCost = False
            continue
        if readingTraj:
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
            Wydot.append(float(line_comps[12]))
        if readingCost:
            Cost = np.append(Cost, float(line[i]))
    
    Xshct.append(X.copy())
    Yshct.append(Y.copy())
    Zshct.append(Z.copy())
    Yawshct.append(Yaw.copy())
    Vxshct.append(Vx.copy())
    Vyshct.append(Vy.copy())
    Vzshct.append(Vz.copy())
    Wyshct.append(Wy.copy())
    Axshct.append(Ax.copy())
    Ayshct.append(Ay.copy())
    Azshct.append(Az.copy())
    Wydotshct.append(Wydot.copy())
    Cost_Shct = np.append(Cost_Shct, Cost.copy())

    file.close()
###############################################################################################

# READING NLOPT
###############################################################################################
subfiles = [f for f in os.listdir(folder_nlopt) if os.path.isfile(os.path.join(folder_nlopt, f))]
for x in subfiles:
    file = open(os.path.join(folder_nlopt, x),"r") 
    line = file.readlines()
    
    readingTraj = False
    readingCost = False

    X = []
    Y = []
    Z = []
    Yaw = []

    Vx = []
    Vy = []
    Vz = []
    Wy = []

    Ax = []
    Ay = []
    Az = []
    Wydot = []

    Cost = np.array([])
    
    Time = np.array([])
    
    compt_traj = 0

    for i in range(len(line)):
        if "Trajectory" in line[i]:
            readingTraj = True
            readingCost = False
            X.clear()
            Y.clear()
            Z.clear()
            Yaw.clear()
            Vx.clear()
            Vy.clear()
            Vz.clear()
            Wy.clear()
            Ax.clear()
            Ay.clear()
            Az.clear()
            Wydot.clear()
            Cost = np.array([])
            continue
        if "Cost" in line[i]:
            readingTraj = False
            readingCost = True
            continue
        if "Time" in line[i]:
            readingTraj = False
            readingCost = False
            # if compt_traj == 1:
            #     Xinit.append(X.copy())
            #     Yinit.append(Y.copy())
            #     Zinit.append(Z.copy())
            #     Yawinit.append(Yaw.copy())
            #     Vxinit.append(Vx.copy())
            #     Vyinit.append(Vy.copy())
            #     Vzinit.append(Vz.copy())
            #     Wyinit.append(Wy.copy())
            #     Axinit.append(Ax.copy())
            #     Ayinit.append(Ay.copy())
            #     Azinit.append(Az.copy())
            #     Wydotinit.append(Wydot.copy())
            #     Cost_Init = np.append(Cost_Init, Cost.copy())
            compt_traj += 1
            continue
        if readingTraj:
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
            Wydot.append(float(line_comps[12]))
        if readingCost:
            Cost = np.append(Cost, float(line[i]))
    
    Xnlopt.append(X.copy())
    Ynlopt.append(Y.copy())
    Znlopt.append(Z.copy())
    Yawnlopt.append(Yaw.copy())
    Vxnlopt.append(Vx.copy())
    Vynlopt.append(Vy.copy())
    Vznlopt.append(Vz.copy())
    Wynlopt.append(Wy.copy())
    Axnlopt.append(Ax.copy())
    Aynlopt.append(Ay.copy())
    Aznlopt.append(Az.copy())
    Wydotnlopt.append(Wydot.copy())
    Cost_Nlopt = np.append(Cost_Nlopt, Cost.copy())

    file.close()
###############################################################################################

# READING RANDOM KINODYNAMIC SHORTCUT FIXED RADIUS WITH COMBINATORIAL
###############################################################################################
subfiles = [f for f in os.listdir(folder_kinoshct) if os.path.isfile(os.path.join(folder_kinoshct, f))]
for x in subfiles:
    file = open(os.path.join(folder_kinoshct, x),"r") 
    line = file.readlines()
    
    readingTraj = False
    readingCost = False

    X = []
    Y = []
    Z = []
    Yaw = []

    Vx = []
    Vy = []
    Vz = []
    Wy = []

    Ax = []
    Ay = []
    Az = []
    Wydot = []

    Cost = np.array([])

    for i in range(len(line)):
        if "Trajectory" in line[i]:
            readingTraj = True
            readingCost = False
            X.clear()
            Y.clear()
            Z.clear()
            Yaw.clear()
            Vx.clear()
            Vy.clear()
            Vz.clear()
            Wy.clear()
            Ax.clear()
            Ay.clear()
            Az.clear()
            Wydot.clear()
            Cost = np.array([])
            continue
        if "Cost" in line[i]:
            readingTraj = False
            readingCost = True
            continue
        if "Time" in line[i]:
            readingTraj = False
            readingCost = False
            continue
        if readingTraj:
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
            Wydot.append(float(line_comps[12]))
        if readingCost:
            Cost = np.append(Cost, float(line[i]))
    
    Xkinoshct_fixed_combi.append(X.copy())
    Ykinoshct_fixed_combi.append(Y.copy())
    Zkinoshct_fixed_combi.append(Z.copy())
    Yawkinoshct_fixed_combi.append(Yaw.copy())
    Vxkinoshct_fixed_combi.append(Vx.copy())
    Vykinoshct_fixed_combi.append(Vy.copy())
    Vzkinoshct_fixed_combi.append(Vz.copy())
    Wykinoshct_fixed_combi.append(Wy.copy())
    Axkinoshct_fixed_combi.append(Ax.copy())
    Aykinoshct_fixed_combi.append(Ay.copy())
    Azkinoshct_fixed_combi.append(Az.copy())
    Wydotkinoshct_fixed_combi.append(Wydot.copy())
    Cost_KinoShct_fixed_combi = np.append(Cost_KinoShct_fixed_combi, Cost.copy())

    file.close()
###############################################################################################

fig1, ax1 = plt.subplots(1)

for i in range(len(Xinit)):
    ax1.plot(Xinit[i], Yinit[i], color='b', linestyle = '-', marker = '', linewidth=2.0)
for i in range(len(Xshct)):
    ax1.plot(Xshct[i], Yshct[i], color='r', linestyle = '-', marker = '', linewidth=2.0)
for i in range(len(Xkinoshct_fixed_combi)):
    ax1.plot(Xkinoshct_fixed_combi[i], Ykinoshct_fixed_combi[i], color='g', linestyle = '-', marker = '', linewidth=2.0)
for i in range(len(Xnlopt)):
    ax1.plot(Xnlopt[i], Ynlopt[i], color='g', linestyle = '-', marker = '', linewidth=2.0)
plt.grid()
ax1.tick_params(axis='both', which='major', labelsize=60)

fig3, ax3 = plt.subplots(1)

for i in range(len(Vxinit)):
    Vinit = np.empty((0,len(Vxinit[i])))
    Vinit = np.vstack((Vinit, Vxinit[i]))
    Vinit = np.vstack((Vinit, Vyinit[i]))
    Vinit = np.vstack((Vinit, Vzinit[i]))
    V_norm_init = np.linalg.norm(Vinit, axis=0)
    
    ax3.plot(np.linspace(0,len(Vxinit[i]),len(Vxinit[i])), V_norm_init, color='b', linestyle = '-', marker = '', linewidth=2.0)

for i in range(len(Vxshct)):
    Vshct = np.empty((0,len(Vxshct[i])))
    Vshct = np.vstack((Vshct, Vxshct[i]))
    Vshct = np.vstack((Vshct, Vxshct[i]))
    Vshct = np.vstack((Vshct, Vxshct[i]))
    V_norm_shct = np.linalg.norm(Vshct, axis=0)
    
    ax3.plot(np.linspace(0,len(Vxshct[i]),len(Vxshct[i])), V_norm_shct, color='r', linestyle = '-', marker = '', linewidth=2.0)

for i in range(len(Vxkinoshct_fixed_combi)): 
    Vkinoshct = np.empty((0,len(Vxkinoshct_fixed_combi[i])))
    Vkinoshct = np.vstack((Vkinoshct, Vxkinoshct_fixed_combi[i]))
    Vkinoshct = np.vstack((Vkinoshct, Vxkinoshct_fixed_combi[i]))
    Vkinoshct = np.vstack((Vkinoshct, Vxkinoshct_fixed_combi[i]))
    V_norm_kinoshct = np.linalg.norm(Vkinoshct, axis=0)
    ax3.plot(np.linspace(0,len(Vxkinoshct_fixed_combi[i]),len(Vxkinoshct_fixed_combi[i])), V_norm_kinoshct, color='g', linestyle = '-', marker = '', linewidth=2.0)
    
plt.grid()
ax3.tick_params(axis='both', which='major', labelsize=60)

planner_costs = [
    Cost_Init,
    Cost_Shct,
    Cost_KinoShct_fixed_combi,
    Cost_Nlopt,
]
labels = ['Init', 'Shct', 'Kino_Shct', 'Nlopt']
colors = ['peachpuff', 'orange', 'tomato', 'salmon']

fig2, ax2 = plt.subplots()
ax2.set_ylabel('Trajectory cost', fontsize = 35)
plt.tick_params(axis='both', which='major', labelsize=25)

bplot = ax2.boxplot(planner_costs,
                   patch_artist=True,  # fill with color
                   labels = labels)  # will be used to label x-ticks

# fill with colors
for patch, color in zip(bplot['boxes'], colors):
    patch.set_facecolor(color)
    
fig4, ax4 = plt.subplots(1)

plt.show() # affiche la figure a l'ecran

print('Done')
###############################################################################################