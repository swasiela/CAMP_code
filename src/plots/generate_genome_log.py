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
import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from math import *

# INITIALIZATION
###############################################################################################
if len(sys.argv) < 1:
	print ("No arguments found. Please specify a robust_traj file to read.")
    
filename = sys.argv[1]
###############################################################################################

# READING TRAJ FILE
###############################################################################################
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

rx = []
ry = []
rz = []

Index = []

file = open(filename,"r") 
line = file.readlines()

readingTraj = False
readingUncertainty = False
readingIndex = False

for i in range(len(line)):
    if "Trajectory" in line[i]:
        readingTraj = True
        readingUncertainty = False
        readingIndex = False
        continue
    if "Uncertainties" in line[i]:
        readingTraj = False
        readingUncertainty = True
        readingIndex = False
        continue
    if "Index Waypoints" in line[i]:
        readingTraj = False
        readingUncertainty = False
        readingIndex = True
        continue
    if readingTraj:
        line_comps = (line[i].split(";"))
        X.append(float(line_comps[1]))
        Y.append(float(line_comps[2]))
        Z.append(float(line_comps[3]))
        R.append(float(line_comps[4]))
        P.append(float(line_comps[5]))
        Yaw.append(float(line_comps[6]))
        Vx.append(float(line_comps[7]))
        Vy.append(float(line_comps[8]))
        Vz.append(float(line_comps[9]))
        Wy.append(float(line_comps[10]))
        Ax.append(float(line_comps[11]))
        Ay.append(float(line_comps[12]))
        Az.append(float(line_comps[13]))
    if readingUncertainty:
        line_comps = (line[i].split(";"))
        rx.append(float(line_comps[1]))
        ry.append(float(line_comps[2]))
        rz.append(float(line_comps[3]))
    if readingIndex:
        line_comps = (line[i].split(";"))
        Index.append(int(line_comps[1]))

file.close()
###############################################################################################

file = open("genom.log","w") 
dt = 0
file.write('ts x y z roll pitch yaw vx vy vz wx wy wz ax ay az dwx dwy dwz jx jy jz ddwx ddwy ddwz\n')
for i in range(len(X)):
    file.write(str(dt)+' '+str(X[i])+' '+str(Y[i])+' '+str(Z[i])+' '+str(R[i])+' '+str(P[i])+' '+str(Yaw[i])+' '+str(Vx[i])+' '+str(Vy[i])+' '+str(Vz[i])+' '+str(0.0)+' '+str(0.0)+' '+str(Wy[i])+' '+str(Ax[i])+' '+str(Ay[i])+' '+str(Az[i])+' '+str(0.0)+' '+str(0.0)+' '+str(0.0)+' '+str(0.0)+' '+str(0.0)+' '+str(0.0)+' '+str(0.0)+' '+str(0.0)+' '+str(0.0)+'\n')
    dt += 0.05
file.close()
