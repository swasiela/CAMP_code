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
import numpy as np
import torch
import torch.nn as nn
import time
import seaborn as sns
import pandas as pd
import shutil
import random

from pathlib import Path
from matplotlib import pyplot as plt
from math import *

# Get the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))

MODE = "QuadTubes" # UnicTubes QuadTubes

if __name__ == "__main__":
    
    ##########################################################################
    # Load from a folder
    ##########################################################################

    # relative_folder = os.path.join('src', 'learning_sensitivity', 'npy_files', 'pre_process')
    # folder = os.path.join(camp_dir, relative_folder)
    
    # gains_dict = {}
    # output_dict = {}
    # input_dict = {}
    
    # outputs_list = []
    # gains_list = []
    # inputs_list = []
    # for file_path in (f.path for f in os.scandir(folder) if f.is_file()):
    #     f_basename = Path(file_path).stem
    #     if 'gains' in file_path:
    #         i = int(f_basename.split('_')[1])
    #         gains_dict[i]=file_path
    #     if 'output' in file_path:
    #         i = int(f_basename.split('_')[1])
    #         output_dict[i]=file_path
    #     if 'input' in file_path:
    #         i = int(f_basename.split('_')[1])
    #         input_dict[i]=file_path
    
    # for i in sorted(input_dict):
    #     inputs_list.append(np.load(input_dict[i]))
    
    # for i in sorted(output_dict):
    #     outputs_list.append(np.load(output_dict[i]))
    
    # for i in sorted(gains_dict):
    #     gains_list.append(np.load(gains_dict[i]))
        
    # gains = np.concatenate(gains_list)
    # outputs = np.concatenate(outputs_list)
    # inputs = np.concatenate(inputs_list)
    
    ##########################################################################
    # Load from single files
    ##########################################################################
    
    # relative_in = os.path.join('src', 'results', 'Unicycle', 'Learning', 'No_Acc', '10k_15m_Tl1m', 'input.npy')
    # relative_out = os.path.join('src', 'results', 'Unicycle', 'Learning', 'No_Acc', '10k_15m_Tl1m', 'output.npy')
    
    relative_in = os.path.join('src', 'results', 'Quad_Learning', '10K_15s_tl1_RK2', 'input.npy')
    relative_out = os.path.join('src', 'results', 'Quad_Learning', '10K_15s_tl1_RK2', 'output.npy')

    path_in = os.path.join(camp_dir, relative_in)
    path_out = os.path.join(camp_dir, relative_out)
    
    inputs = np.load(path_in)
    outputs = np.load(path_out)
    
    ##########################################################################
    # Reshape and build dict
    ##########################################################################
    if MODE == 'QuadTubes':
        
        # Plot each control input as subplots
        index = random.randint(0, 1000) # Get a random trajectory from the test or validation set
        num_controls = 4
        fig, axes = plt.subplots(4, 1, figsize=(10, 2 * 4), sharex=True)
        axes[0].plot(np.linspace(0,len(outputs[index,:,3]), len(outputs[index,:,3])), outputs[index,:,3], label="u1", linestyle='--')
        axes[1].plot(np.linspace(0,len(outputs[index,:,3]), len(outputs[index,:,3])), outputs[index,:,4], label="u2", linestyle='--')
        axes[2].plot(np.linspace(0,len(outputs[index,:,3]), len(outputs[index,:,3])), outputs[index,:,5], label="u3", linestyle='--')
        axes[3].plot(np.linspace(0,len(outputs[index,:,3]), len(outputs[index,:,3])), outputs[index,:,6], label="u4", linestyle='--')
        plt.tight_layout(rect=[0, 0, 1, 0.97])  # Adjust layout to fit the title
        
        num_rq = 3
        fig1, axes1 = plt.subplots(3, 1, figsize=(10, 2 * 4), sharex=True)
        axes1[0].plot(np.linspace(0,len(outputs[index,:,0]), len(outputs[index,:,0])), outputs[index,:,0], label="rx", linestyle='--')
        axes1[1].plot(np.linspace(0,len(outputs[index,:,1]), len(outputs[index,:,1])), outputs[index,:,1], label="ry", linestyle='--')
        axes1[2].plot(np.linspace(0,len(outputs[index,:,2]), len(outputs[index,:,2])), outputs[index,:,2], label="rz", linestyle='--')
        plt.tight_layout(rect=[0, 0, 1, 0.97])  # Adjust layout to fit the title
        plt.show()
        
        Vx = np.reshape(inputs[:,:,0],(1,-1))[0]
        Vy = np.reshape(inputs[:,:,1],(1,-1))[0]
        Vz = np.reshape(inputs[:,:,2],(1,-1))[0]
        Wyaw = np.reshape(inputs[:,:,3],(1,-1))[0]
        
        Ax = np.reshape(inputs[:,:,4],(1,-1))[0]
        Ay = np.reshape(inputs[:,:,5],(1,-1))[0]
        Az = np.reshape(inputs[:,:,6],(1,-1))[0]
        
        V = np.empty((0,len(Vx)))
        V = np.vstack((V, Vx))
        V = np.vstack((V, Vy))
        V = np.vstack((V, Vz))
        V_norm = np.linalg.norm(V, axis=0)
        
        A = np.empty((0,len(Ax)))
        A = np.vstack((A, Ax))
        A = np.vstack((A, Ay))
        A = np.vstack((A, Az))
        A_norm = np.linalg.norm(A, axis=0)
        
        rx = np.reshape(outputs[:,:,0],(1,-1))[0]
        ry = np.reshape(outputs[:,:,1],(1,-1))[0]
        rz = np.reshape(outputs[:,:,2],(1,-1))[0]
        
        u1 = np.reshape(outputs[:,:,3],(1,-1))[0]
        u2 = np.reshape(outputs[:,:,4],(1,-1))[0]
        u3 = np.reshape(outputs[:,:,5],(1,-1))[0]
        u4 = np.reshape(outputs[:,:,6],(1,-1))[0]
        
        ru1 = np.reshape(outputs[:,:,7],(1,-1))[0]
        ru2 = np.reshape(outputs[:,:,8],(1,-1))[0]
        ru3 = np.reshape(outputs[:,:,9],(1,-1))[0]
        ru4 = np.reshape(outputs[:,:,10],(1,-1))[0]
        
        rq_max = max(rx.max(), max(ry.max(),rz.max()))
        u_max = max(u1.max(), max(u2.max(), max(u3.max(),u4.max())))
        ru_max = max(ru1.max(), max(ru2.max(), max(ru3.max(),ru4.max())))
        
        print("Max rq", rq_max)
        print("Max u", u_max)
        print("Max ru", ru_max)
        
        Rq = np.empty((0,len(rx)))
        Rq = np.vstack((Rq, rx))
        Rq = np.vstack((Rq, ry))
        Rq = np.vstack((Rq, rz))
        Rq_norm = np.linalg.norm(Rq, axis=0)
        
        U = np.empty((0,len(u1)))
        U = np.vstack((U, u1))
        U = np.vstack((U, u2))
        U = np.vstack((U, u3))
        U = np.vstack((U, u4))
        U_norm = np.linalg.norm(U, axis=0)
        
        Ru = np.empty((0,len(ru1)))
        Ru = np.vstack((Ru, ru1))
        Ru = np.vstack((Ru, ru2))
        Ru = np.vstack((Ru, ru3))
        Ru = np.vstack((Ru, ru4))
        Ru_norm = np.linalg.norm(Ru, axis=0)
        
        dv = {'Vx': Vx, 'Vy': Vy, 'Vz': Vz, 'Wyaw': Wyaw, 'V_norm': V_norm}
        da = {'Ax': Ax, 'Ay': Ay, 'Az': Az, 'A_norm': A_norm}
        drq = {'rx': rx, 'ry': ry, 'rz': rz}
        dru = {'ru1': ru1, 'ru2': ru2, 'ru3': ru3, 'ru4': ru4}
        du = {'u1': u1, 'u2': u2, 'u3': u3, 'u4': u4} 
        dfv = pd.DataFrame(data=dv)  
        dfa = pd.DataFrame(data=da)
        dfrq = pd.DataFrame(data=drq)
        dfru = pd.DataFrame(data=dru)
        dfu = pd.DataFrame(data=du)
        
        sns.set(font_scale = 5, style='white')
        sns.displot(data=dfv['Vx'], kind="hist", kde=False)
        plt.yscale('log')
        plt.xlabel("m/s", fontsize=55)
        plt.ylabel("Count", fontsize=55)
        plt.grid() 
        plt.ylim(1e0,1e6)
        plt.show()
            
    elif MODE == "UnicTubes":
        Vx = np.reshape(inputs[:,:,0],(1,-1))[0]
        Vy = np.reshape(inputs[:,:,1],(1,-1))[0]
        
        V = np.empty((0,len(Vx)))
        V = np.vstack((V, Vx))
        V = np.vstack((V, Vy))
        V_norm = np.linalg.norm(V, axis=0)
        
        rx = np.reshape(outputs[:,:,0],(1,-1))[0]
        ry = np.reshape(outputs[:,:,1],(1,-1))[0]
        
        u1 = np.reshape(outputs[:,:,2],(1,-1))[0]
        u2 = np.reshape(outputs[:,:,3],(1,-1))[0]
        
        ru1 = np.reshape(outputs[:,:,4],(1,-1))[0]
        ru2 = np.reshape(outputs[:,:,5],(1,-1))[0]
        
        rq_max = max(rx.max(),ry.max())
        u_max = max(u1.max(), u2.max())
        ru_max = max(ru1.max(), ru2.max())
        
        print("Max rq", rq_max)
        print("Max u", u_max)
        print("Max ru", ru_max)
        
        Rq = np.empty((0,len(rx)))
        Rq = np.vstack((Rq, rx))
        Rq = np.vstack((Rq, ry))
        Rq_norm = np.linalg.norm(Rq, axis=0)
        
        U = np.empty((0,len(u1)))
        U = np.vstack((U, u1))
        U = np.vstack((U, u2))
        U_norm = np.linalg.norm(U, axis=0)
        
        Ru = np.empty((0,len(ru1)))
        Ru = np.vstack((Ru, ru1))
        Ru = np.vstack((Ru, ru2))
        Ru_norm = np.linalg.norm(Ru, axis=0)
        
        dv = {'Vx': Vx, 'Vy': Vy, 'V_norm': V_norm}
        drq = {'rx': rx, 'ry': ry}
        dru = {'ru1': ru1, 'ru2': ru2}
        du = {'u1': u1, 'u2': u2} 
        dfv = pd.DataFrame(data=dv)  
        dfrq = pd.DataFrame(data=drq)
        dfru = pd.DataFrame(data=dru)
        dfu = pd.DataFrame(data=du)
        
        sns.set(font_scale = 5, style='white')
        sns.displot(data=dfu['u1'], kind="hist", kde=False)
        plt.yscale('log')
        plt.xlabel("m/s", fontsize=55)
        plt.ylabel("Count", fontsize=55)
        plt.grid() 
        plt.ylim(1e0,1e6)
        plt.show()
        
    print("Done")
