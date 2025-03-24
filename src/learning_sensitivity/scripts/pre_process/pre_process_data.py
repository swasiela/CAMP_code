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

import torch
import pandas as pd
import os
import shutil
import numpy as np
import cv2
import torch.utils.data as data
from utils import *
from random import seed, sample
from sklearn.preprocessing import StandardScaler

from math import *

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..'))

MODE = 'QuadTubes' # 'QuadTubes' 'UnicTubes'

relative_archive = os.path.join('src', 'learning_sensitivity', 'npy_files', 'pre_process')
archive_path = os.path.join(camp_dir, relative_archive)

relative_file = os.path.join('src', 'results', 'Quad_Learning_FIX', 'RK2', 'Classic_V2', '10K_15s_tl1.csv')
file_ = os.path.join(camp_dir, relative_file)

nb_split = 1

seed(0)

#############################################################################################
def string2float(x):
    x = x.split(' ')[:-1]
    for i in range(0,len(x)):
        x[i] = float(x[i])
    return x

#############################################################################################  
class PreProcessing():
    def __init__(self, mode):
        
        # Create the output folder if it dosn't exist
        os.makedirs(archive_path, exist_ok=True)
        
        self.min_traj_len = 1000000
        
        self.mode = mode
        
        if self.mode == 'QuadTubes': 
            inputs_tag = ['Vx', 'Vy', 'Vz', 'Wyaw', 'Ax', 'Ay', 'Az'] 
            targets_tag = ['rx', 'ry', 'rz','u1', 'u2', 'u3', 'u4', 'ru1', 'ru2', 'ru3', 'ru4']
        
        elif self.mode == 'UnicTubes':
            inputs_tag = ['Vx', 'Vy'] 
            targets_tag = ['rx', 'ry','u1', 'u2', 'ru1', 'ru2']
            
        else:
            print("Mode not implemented yet !")
            
        self.nb_inputs = len(inputs_tag)
        self.nb_outputs = len(targets_tag)
            
        ########################################################################################## 
        # Pre process inputs 
        ########################################################################################## 
        
        self.pre_process_inputs(inputs_tag)
        print("Input data succesfully pre process !")
        
        ########################################################################################## 
        # Pre process inputs
        ########################################################################################## 
        
        self.pre_process_outputs(targets_tag)
        print("Output data succesfully pre process !")
    
    def pre_process_inputs(self, inputs_tag):
        
        inputs_csv = pd.read_csv(file_, usecols=inputs_tag)
        
        for tag in inputs_tag:
            inputs_csv[tag] = inputs_csv[tag].apply(string2float)
            
        # Get the minimum length of a trajectory for the cut
        V_Xs = inputs_csv.Vx
        for V_X_traj in V_Xs:
            if len(V_X_traj) < self.min_traj_len:
                self.min_traj_len = len(V_X_traj)
        
        # Split the export 
        for i in range(0, nb_split):
            idx_start = int(i*(len(inputs_csv)/nb_split))
            idx_end = int((i+1)*(len(inputs_csv)/nb_split))
            idx = (idx_start, idx_end)
            input_NN = self.get_input_from_csv(inputs_csv, idx)
            self.generate_input_file(input_NN, (idx_end-idx_start), i)
    
    def pre_process_outputs(self, targets_tag):
        
        targets_csv = pd.read_csv(file_, usecols=targets_tag)
        
        for tag in targets_tag:
            targets_csv[tag] = targets_csv[tag].apply(string2float)
            
        # Split the export 
        for i in range(0, nb_split):
            idx_start = int(i*(len(targets_csv)/nb_split))
            idx_end = int((i+1)*(len(targets_csv)/nb_split))
            idx = (idx_start, idx_end)
            target_NN = self.get_output_from_csv(targets_csv, idx)
            self.generate_output_file(target_NN, (idx_end-idx_start), i)
    
    def get_input_from_csv(self, inputs_csv, idx):
        
        idx0, idx1 = idx
        
        if self.mode == 'QuadTubes': 
            #input_NN: Vx, Vy, Vz, Wyaw, Ax, Ay, Az
            input_NN = [[] for i in range(0,self.nb_inputs)]
        elif self.mode == 'UnicTubes': 
            #input_NN: Vx, Vy, Ax, Ay
            input_NN = [[] for i in range(0,self.nb_inputs)]
        
        for j in range(idx0,idx1):
            print("Processing traj inputs ", j)

            for i in range(0,self.min_traj_len-1):
                
                if self.mode == 'QuadTubes': 
                    input_NN[0].append(inputs_csv.loc[j].Vx[i])
                    input_NN[1].append(inputs_csv.loc[j].Vy[i])
                    input_NN[2].append(inputs_csv.loc[j].Vz[i])
                    input_NN[3].append(inputs_csv.loc[j].Wyaw[i])
                    input_NN[4].append(inputs_csv.loc[j].Ax[i])
                    input_NN[5].append(inputs_csv.loc[j].Ay[i])
                    input_NN[6].append(inputs_csv.loc[j].Az[i])
                
                elif self.mode == 'UnicTubes':
                
                    input_NN[0].append(inputs_csv.loc[j].Vx[i])
                    input_NN[1].append(inputs_csv.loc[j].Vy[i])
                    # input_NN[2].append(inputs_csv.loc[j].Ax[i])
                    # input_NN[3].append(inputs_csv.loc[j].Ay[i])
                       
        return input_NN
    
    def get_output_from_csv(self, targets_csv, idx):
        
        idx0, idx1 = idx
        
        if self.mode == 'QuadTubes': 
            target_NN = [[] for i in range(0,self.nb_outputs)]
        elif self.mode == 'UnicTubes': 
            target_NN = [[] for i in range(0,self.nb_outputs)]
        
        for j in range(idx0,idx1):
            print("Processing output ", j)
    
            for i in range(0,self.min_traj_len-1):
                
                if self.mode == 'QuadTubes': 
                    target_NN[0].append(targets_csv.loc[j].rx[i])
                    target_NN[1].append(targets_csv.loc[j].ry[i])
                    target_NN[2].append(targets_csv.loc[j].rz[i])
                    target_NN[3].append(targets_csv.loc[j].u1[i])
                    target_NN[4].append(targets_csv.loc[j].u2[i])
                    target_NN[5].append(targets_csv.loc[j].u3[i])
                    target_NN[6].append(targets_csv.loc[j].u4[i])
                    target_NN[7].append(targets_csv.loc[j].ru1[i])
                    target_NN[8].append(targets_csv.loc[j].ru2[i])
                    target_NN[9].append(targets_csv.loc[j].ru3[i])
                    target_NN[10].append(targets_csv.loc[j].ru4[i])
                
                elif self.mode == 'UnicTubes':
                    target_NN[0].append(targets_csv.loc[j].rx[i])
                    target_NN[1].append(targets_csv.loc[j].ry[i])
                    target_NN[2].append(targets_csv.loc[j].u1[i])
                    target_NN[3].append(targets_csv.loc[j].u2[i])
                    target_NN[4].append(targets_csv.loc[j].ru1[i])
                    target_NN[5].append(targets_csv.loc[j].ru2[i])
                        
        return target_NN
    
    def generate_input_file(self, input_NN, nb_traj, i):
        set_input = []
        
        for j in range(0,nb_traj):
            print("Generating input array ", j)
            inputs = []
            for l in range(0,self.min_traj_len-1):
                index = j*(self.min_traj_len-1)+l
                waypoint_X = []
                for k in range(0,len(input_NN)):
                    waypoint_X.append(input_NN[k][index])
                inputs.append(waypoint_X)
            set_input.append(inputs)

        with open(f'{archive_path}/input_{i}.npy', 'wb') as inpt:
            np.save(inpt, np.array(set_input)) 

    def generate_output_file(self, target_NN, nb_output, i):
        set_output = []
        
        for j in range(0,nb_output):
            outputs = []
            for l in range(0,self.min_traj_len-1):
                index = j*(self.min_traj_len-1)+l
                waypoint_Y = []
                for k in range(0,len(target_NN)):
                    waypoint_Y.append(target_NN[k][index])
                outputs.append(waypoint_Y)
            set_output.append(outputs)

        with open(f'{archive_path}/output_{i}.npy', 'wb') as out:
            np.save(out, np.array(set_output))                          
#############################################################################################   
 
def pre_processing_data(mode):
    dataset = PreProcessing(mode)
    print("Data succesfully pre process !")
#############################################################################################  
    
if __name__ == "__main__":
    pre_processing_data(MODE)
