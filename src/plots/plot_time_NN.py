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

from os import listdir
from os.path import isfile, join
from scipy.interpolate import interp1d
import scipy.optimize

import subprocess
import sys
import os
import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from math import *

# INITIALIZATION
###############################################################################################

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))

relative_filename = os.path.join('src', 'results', 'Iterations', 'no_star', 'RRT.txt')
relative_filename2 = os.path.join('src', 'results', 'Iterations', 'no_star', 'R_SARRT_NN.txt')
relative_filename3 = os.path.join('src', 'results', 'Iterations', 'no_star', 'R_SARRT.txt')

filename = os.path.join(camp_dir, relative_filename)
filename2 = os.path.join(camp_dir, relative_filename2)
filename3 = os.path.join(camp_dir, relative_filename3)

Nb_iter_NN = []
Time_NN = []
NbNode_NN = []
Time_added_NN = []

Nb_iter = []
Time = []
NbNode = []
Time_added = []

Nb_iter_classic = []
Time_classic = []
NbNode_classic = []
Time_added_classic = []

rescale_time = 1e6

###############################################################################################

# READING TRAJ FILE
###############################################################################################
file = open(filename2,"r") 
line = file.readlines()

readingIter = False
readingNode = False

for i in range(len(line)):
    if "Iteration" in line[i]:
        readingIter = True
        readingNode = False
    if "NbNode" in line[i]:
        readingIter = False
        readingNode = True
    if readingIter:
        line_comps = (line[i].split(";"))
        Nb_iter_NN.append(float((line_comps[0].split(":"))[1]))
        Time_NN.append(float((line_comps[1].split(":"))[1]))
    if readingNode:
        line_comps = (line[i].split(";"))
        NbNode_NN.append(float((line_comps[0].split(":"))[1]))
        Time_added_NN.append(float((line_comps[1].split(":"))[1]))
    
file.close()
###############################################################################################

# READING TRAJ FILE
###############################################################################################
file = open(filename3,"r") 
line = file.readlines()

readingIter = False
readingNode = False

for i in range(len(line)):
    if "Iteration" in line[i]:
        readingIter = True
        readingNode = False
    if "NbNode" in line[i]:
        readingIter = False
        readingNode = True
    if readingIter:
        line_comps = (line[i].split(";"))
        Nb_iter.append(float((line_comps[0].split(":"))[1]))
        Time.append(float((line_comps[1].split(":"))[1]))
    if readingNode:
        line_comps = (line[i].split(";"))
        NbNode.append(float((line_comps[0].split(":"))[1]))
        Time_added.append(float((line_comps[1].split(":"))[1]))
    
file.close()
###############################################################################################

# READING TRAJ FILE
###############################################################################################
file = open(filename,"r") 
line = file.readlines()

readingIter = False
readingNode = False

for i in range(len(line)):
    if "Iteration" in line[i]:
        readingIter = True
        readingNode = False
    if "NbNode" in line[i]:
        readingIter = False
        readingNode = True
    if readingIter:
        line_comps = (line[i].split(";"))
        Nb_iter_classic.append(float((line_comps[0].split(":"))[1]))
        Time_classic.append(float((line_comps[1].split(":"))[1]))
    if readingNode:
        line_comps = (line[i].split(";"))
        NbNode_classic.append(float((line_comps[0].split(":"))[1]))
        Time_added_classic.append(float((line_comps[1].split(":"))[1]))
    
file.close()
###############################################################################################

fig1, ax1 = plt.subplots(1)

ax1.plot([time/rescale_time for time in Time[:]], Nb_iter[:], color='r', linestyle = '-', marker = '', linewidth=4.0)
ax1.plot([time/rescale_time for time in Time_NN[:]], Nb_iter_NN[:], color='g', linestyle = '-', marker = '', linewidth=4.0)
ax1.plot([time/rescale_time for time in Time_classic[:]], Nb_iter_classic[:], color='b', linestyle = '-', marker = '', linewidth=4.0)
ax1.set_xlabel('Planning time (s)', fontsize = 60)
ax1.set_ylabel('Iteration number',fontsize = 60)

ticks = np.arange(0, 20000, 2500)
ax1.set_yticks(ticks)
plt.grid()

ax1.tick_params(axis='both', which='major', labelsize=60)

plt.show() # affiche la figure a l'ecran

print("SARRT_NN is faster than SARRT by a factor: ",Time[1500]/Time_NN[1500])
print("RRT is faster than SARRT_NN by a factor: ",Time_NN[4000]/Time_classic[4000])
###############################################################################################