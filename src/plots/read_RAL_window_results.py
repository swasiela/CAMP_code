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
import os
import glob

import subprocess
import sys
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

relative_foldername= os.path.join('src', 'results', 'RAL2024', 'window', 'RRTstar')
foldername = os.path.join(camp_dir, relative_foldername)

time = []
success = 0
nb_tot = 0

subfolders = [x[0] for x in os.walk(foldername)]
for x in subfolders:
    if "Exp_" in x:
        continue
    if "Traj_" in x:
        nb_tot += 30
        file_list = glob.glob(x+"/*.txt")
        for f in file_list:
            if "results" in f:
                file = open(f,"r") 
                line = file.readlines()

                for i in range(len(line)):
                    if "Cost" in line[i]:
                        continue
                    if "Time" in line[i]:
                        line_comps = (line[i].split(":"))
                        time.append(float(line_comps[1]))
                    if "Success" in line[i]:
                        line_comps = (line[i].split(":"))
                        success += (float((line_comps[1].split("/"))[0]))          
                file.close()
                
print('Mean time: ', np.mean(time))
print('Std time: ', np.std(time))

print('Success: ', (success/nb_tot) *100)