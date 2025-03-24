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
relative_folder_stomp = os.path.join('src', 'results', 'LocalOpt', 'Methods', 'Stomp')

folder_shct = os.path.join(camp_dir, relative_folder_shct)
folder_nlopt = os.path.join(camp_dir, relative_folder_nlopt)
folder_kinoshct = os.path.join(camp_dir, relative_folder_kinoshct)
folder_stomp = os.path.join(camp_dir, relative_folder_stomp)

# For the Shortcut
Cost_Shct = []
Time_Shct = []

# For the KinoShortcut fixed radius

Cost_KinoShct_fixed_combi = []
Time_KinoShct_fixed_combi = []

# For Nlopt method
Cost_Nlopt = []
Time_Nlopt = []

# For STOMP method
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
    readingTime = False

    for i in range(len(line)):
        if "Trajectory" in line[i]:
            readingTraj = True
            readingCost = False
            readingTime = False
            continue
        if "Cost" in line[i]:
            readingTraj = False
            readingCost = True
            readingTime = False
            continue
        if "Time" in line[i]:
            readingTraj = False
            readingCost = False
            readingTime = True
            continue
        if readingTraj:
            continue
        if readingCost:
            Cost_Shct.append(float(line[i]))
        if readingTime:
            Time_Shct.append(float(line[i]))

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
    readingTime = False

    for i in range(len(line)):
        if "Trajectory" in line[i]:
            readingTraj = True
            readingCost = False
            readingTime = False
            continue
        if "Cost" in line[i]:
            readingTraj = False
            readingCost = True
            readingTime = False
            continue
        if "Time" in line[i]:
            readingTraj = False
            readingCost = False
            readingTime = True
            continue
        if readingTraj:
            continue
        if readingCost:
            Cost_Nlopt.append(float(line[i]))
        if readingTime:
            Time_Nlopt.append(float(line[i]))

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
    readingTime = False

    for i in range(len(line)):
        if "Trajectory" in line[i]:
            readingTraj = True
            readingCost = False
            readingTime = False
            continue
        if "Cost" in line[i]:
            readingTraj = False
            readingCost = True
            readingTime = False
            continue
        if "Time" in line[i]:
            readingTraj = False
            readingCost = False
            readingTime = True
            continue
        if readingTraj:
            continue
        if readingCost:
            Cost_KinoShct_fixed_combi.append(float(line[i]))
        if readingTime:
            Time_KinoShct_fixed_combi.append(float(line[i]))

    file.close()
###############################################################################################

# READING STOMP
###############################################################################################
subfiles = [f for f in os.listdir(folder_stomp) if os.path.isfile(os.path.join(folder_stomp, f))]
for x in subfiles:
    file = open(os.path.join(folder_stomp, x),"r") 
    line = file.readlines()
    
    readingTraj = False
    readingCost = False
    readingTime = False

    for i in range(len(line)):
        if "Trajectory" in line[i]:
            readingTraj = True
            readingCost = False
            readingTime = False
            continue
        if "Cost" in line[i]:
            readingTraj = False
            readingCost = True
            readingTime = False
            continue
        if "Time" in line[i]:
            readingTraj = False
            readingCost = False
            readingTime = True
            continue
        if readingTraj:
            continue
        if readingCost:
            Cost_stomp.append(float(line[i]))
        if readingTime:
            Time_stomp.append(float(line[i]))

    file.close()
###############################################################################################

fig1, ax1 = plt.subplots(1)

ax1.plot(Time_Nlopt, Cost_Nlopt, color='b', linestyle = '-', marker = '', linewidth=2.0, label="Nlopt")
ax1.plot(Time_Shct, Cost_Shct, color='g', linestyle = '-', marker = '', linewidth=2.0, label="Shortcut")
ax1.plot(Time_KinoShct_fixed_combi, Cost_KinoShct_fixed_combi, color='r', linestyle = '-', marker = '', linewidth=2.0, label="Steady shorcut")
ax1.plot(Time_stomp, Cost_stomp, color='k', linestyle = '-', marker = '', linewidth=2.0, label="STOMP")

ax1.set_xlabel('Planning time (s)')
ax1.set_ylabel('Cost')
ax1.legend()
ax1.grid()

plt.show() # affiche la figure a l'ecran

print('Done')
###############################################################################################