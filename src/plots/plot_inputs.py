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
	print ("No arguments found. Please specify an input file.")
else:
	filename = sys.argv[1]


U1 = []
U2 = []
U3 = []
U4 = []

ellips_U1 = []
ellips_U2 = []
ellips_U3 = []
ellips_U4 = []
###############################################################################################


# READING INPUT FILE
###############################################################################################
file = open(filename,"r") 
line = file.readlines()

readingU1 = False
readingU2 = False
readingU3 = False
readingU4 = False
readingElU1 = False
readingElU2 = False
readingElU3 = False
readingElU4 = False


for i in range(len(line)):
    if "Input1" in line[i]:
        readingU1 = True
        readingU2 = False
        readingU3 = False
        readingU4 = False
        readingElU1 = False
        readingElU2 = False
        readingElU3 = False
        readingElU4 = False
        continue
    if "Input2" in line[i]:
        readingU1 = False
        readingU2 = True
        readingU3 = False
        readingU4 = False
        readingElU1 = False
        readingElU2 = False
        readingElU3 = False
        readingElU4 = False
        continue
    if "Input3" in line[i]:
        readingU1 = False
        readingU2 = False
        readingU3 = True
        readingU4 = False
        readingElU1 = False
        readingElU2 = False
        readingElU3 = False
        readingElU4 = False
        continue
    if "Input4" in line[i]:
        readingU1 = False
        readingU2 = False
        readingU3 = False
        readingU4 = True
        readingElU1 = False
        readingElU2 = False
        readingElU3 = False
        readingElU4 = False
        continue
    if "Input4" in line[i]:
        readingU1 = False
        readingU2 = False
        readingU3 = False
        readingU4 = True
        readingElU1 = False
        readingElU2 = False
        readingElU3 = False
        readingElU4 = False
        continue
    if "U1 ellipsoid" in line[i]:
        readingU1 = False
        readingU2 = False
        readingU3 = False
        readingU4 = False
        readingElU1 = True
        readingElU2 = False
        readingElU3 = False
        readingElU4 = False
        continue
    if "U2 ellipsoid" in line[i]:
        readingU1 = False
        readingU2 = False
        readingU3 = False
        readingU4 = False
        readingElU1 = False
        readingElU2 = True
        readingElU3 = False
        readingElU4 = False
        continue
    if "U3 ellipsoid" in line[i]:
        readingU1 = False
        readingU2 = False
        readingU3 = False
        readingU4 = False
        readingElU1 = False
        readingElU2 = False
        readingElU3 = True
        readingElU4 = False
        continue
    if "U4 ellipsoid" in line[i]:
        readingU1 = False
        readingU2 = False
        readingU3 = False
        readingU4 = False
        readingElU1 = False
        readingElU2 = False
        readingElU3 = False
        readingElU4 = True
        continue
   
    if readingU1:
        line_comps = line[i].split(";")
        U1.append(float(line_comps[0]))
    if readingU2:
        line_comps = line[i].split(";")
        U2.append(float(line_comps[0]))
    if readingU3:
        line_comps = line[i].split(";")
        U3.append(float(line_comps[0]))
    if readingU4:
        line_comps = line[i].split(";")
        U4.append(float(line_comps[0]))
    if readingElU1:
        line_comps = line[i].split(";")
        ellips_U1.append(float(line_comps[0]))
    if readingElU2:
        line_comps = line[i].split(";")
        ellips_U2.append(float(line_comps[0]))
    if readingElU3:
        line_comps = line[i].split(";")
        ellips_U3.append(float(line_comps[0]))
    if readingElU4:
        line_comps = line[i].split(";")
        ellips_U4.append(float(line_comps[0]))

file.close()
###############################################################################################

# PLOT
###############################################################################################
fig1, (ax11, ax12, ax13, ax14) = plt.subplots(4)
fig2, ax2 = plt.subplots(1)

max_input = []
min_input = []
time_vec = []
U1_upper = []
U1_lower = []
U2_upper = []
U2_lower = []
U3_upper = []
U3_lower = []
U4_upper = []
U4_lower = []


for i in  range(len(U1)):
    max_input.append(10000)
    min_input.append(0.0)
    time_vec.append(i*0.05)

ax11.plot(time_vec, U1, color='b', linestyle = '-', marker = '', label='U1')
ax12.plot(time_vec, U2, color='b', linestyle = '-', marker = '', label='U2')
ax13.plot(time_vec, U3, color='b', linestyle = '-', marker = '', label='U3')
ax14.plot(time_vec, U4, color='b', linestyle = '-', marker = '', label='U4')

ax11.plot(time_vec, max_input, color='r', linestyle = '--', marker = '', label='U1 maximum input')
ax12.plot(time_vec, max_input, color='r', linestyle = '--', marker = '', label='U2 maximum input')
ax13.plot(time_vec, max_input, color='r', linestyle = '--', marker = '', label='U3 maximum input')
ax14.plot(time_vec, max_input, color='r', linestyle = '--', marker = '', label='U4 maximum input')

for i in  range(len(ellips_U1)):
    U1_upper.append(U1[i]+ellips_U1[i])
    U1_lower.append(U1[i]-ellips_U1[i])
    U2_upper.append(U2[i]+ellips_U2[i])
    U2_lower.append(U2[i]-ellips_U2[i])
    U3_upper.append(U3[i]+ellips_U3[i])
    U3_lower.append(U3[i]-ellips_U3[i])
    U4_upper.append(U4[i]+ellips_U4[i])
    U4_lower.append(U4[i]-ellips_U4[i])


ax11.plot(time_vec, U1_upper, color='g', linestyle = '-', marker = '', label='U1 uncertainty tube')
ax11.plot(time_vec, U1_lower, color='g', linestyle = '-', marker = '', label='')
ax12.plot(time_vec, U2_upper, color='g', linestyle = '-', marker = '', label='U2 uncertainty tube')
ax12.plot(time_vec, U2_lower, color='g', linestyle = '-', marker = '', label='')
ax13.plot(time_vec, U3_upper, color='g', linestyle = '-', marker = '', label='U3 uncertainty tube')
ax13.plot(time_vec, U3_lower, color='g', linestyle = '-', marker = '', label='')
ax14.plot(time_vec, U4_upper, color='g', linestyle = '-', marker = '', label='U4 uncertainty tube')
ax14.plot(time_vec, U4_lower, color='g', linestyle = '-', marker = '', label='')

ax2.plot(time_vec, U1, color='b', linestyle = '-', marker = '', label='U1', linewidth=3.0)
ax2.plot(time_vec, max_input, color='r', linestyle = '--', marker = '', label='U1 maximum input', linewidth=3.0)
ax2.plot(time_vec, min_input, color='r', linestyle = '--', marker = '', label='U1 maximum input', linewidth=3.0)
ax2.plot(time_vec, U1_upper, color='g', linestyle = '-', marker = '', label='U1 uncertainty tube', linewidth=3.0)
ax2.plot(time_vec, U1_lower, color='g', linestyle = '-', marker = '', label='', linewidth=3.0)


ax11.set_ylim([0.0, 11500.0])
ax12.set_ylim([0.0, 11500.0])
ax13.set_ylim([0.0, 11500.0])
ax14.set_ylim([0.0, 11500.0])
ax2.set_ylim([-100, 11500.0])

ax11.set_xlim([0.0, time_vec[-1]+0.05])
ax12.set_xlim([0.0, time_vec[-1]+0.05])
ax13.set_xlim([0.0, time_vec[-1]+0.05])
ax14.set_xlim([0.0, time_vec[-1]+0.05])
ax2.set_xlim([0.0, time_vec[-1]+0.05])

ax11.grid(True,which='both')
ax12.grid(True,which='both')
ax13.grid(True,which='both')
ax14.grid(True,which='both')
ax2.grid(True,which='both')

ax11.set_ylabel('U1 (rpm)', fontsize = 14)
#ax11.set_xlabel('Execution time of the trajectory (s)')
ax12.set_ylabel('U2 (rpm)', fontsize = 14)
#ax12.set_xlabel('Execution time of the trajectory (s)')
ax13.set_ylabel('U3 (rpm)', fontsize = 14)
#ax13.set_xlabel('Execution time of the trajectory (s)',fontsize=8)
ax14.set_ylabel('U4 (rpm)', fontsize = 14)
ax14.set_xlabel('Execution time of the trajectory (s)',fontsize=15)

ax2.set_ylabel('U1 (rpm)', fontsize = 30)
ax2.set_xlabel('Time (s)',fontsize=30)

ax2.tick_params(axis='both', which='major', labelsize=30)

plt.show()
