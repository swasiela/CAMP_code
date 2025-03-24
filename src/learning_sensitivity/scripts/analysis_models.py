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

import numpy as np
import torch
import torch.nn as nn
import time
import seaborn as sns
import pandas as pd
import yaml
import os

from torch.utils.data import DataLoader
from matplotlib import pyplot as plt
import matplotlib.colors as mcolors
from math import *

from TransformerEncoder import NetTransformer, LightningNetTransformer
from GRU import NetGRU, LightningNetGRU
from LSTM import NetLSTM, LightningNetLSTM
from RNN import NetRNN, LightningNetRNN
from MLP import NetMLP, LightningNetMLP

# Get the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))

MODE = "QuadTubes" # UnicTubes QuadTubes

SCALING = 'Std' # Std MinMax

# Min-Max Scaling	Simple, ensures values are bounded to a range.	Compresses amplitude if application range is smaller than training.
# Standardization	Robust to differing ranges, keeps proportionality.

# Values for the min max scaling of the input components
Vmax = 5.0
Wmax = 0.5
Amax = 1.5

Vmin = -Vmax
Wmin = -Wmax
Amin = -Amax

if MODE == "QuadTubes":
    INPUT_SIZE = 7
elif MODE == "UnicTubes":
    INPUT_SIZE = 2
    
BATCH_SIZE = 1000 # One batch for the whole test set

# Set to TRUE the model you want to analyze
RNN = False
GRU = True
LSTM = False
MLP = False
TRANS = False

# TRUE: Use the validation set, FALSE: use the test set
VAL = False

device = "cpu"
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

#############################################################################################
def val_dataloader(inputs, outputs, std_out, mean_out, minmax_in, std_in, mean_in, min_out, max_out, min_in, max_in, index_end):

    inputs_test = inputs.copy()
    outputs_test = outputs.copy()
    
    inputs_test = np.float64(inputs_test[index_end:,:,:])
    outputs_test = np.float64(outputs_test[index_end:,:,:])
    
    # Outputs standardization
    outputs_test[:,:] = (outputs_test[:,:]-mean_out)/std_out
    
    if MODE == "QuadTubes":
        # Inputs scaling
        if SCALING == 'MinMax':
            for i in range(0,7):
                inputs_test[:,:,i] = inputs_test[:,:,i] - minmax_in[2*i] / (minmax_in[2*i+1]-minmax_in[2*i])
        if SCALING == 'Std':
            inputs_test[:,:] = (inputs_test[:,:]-mean_in)/std_in

    elif MODE == "UnicTubes":
        # Inputs scaling
        if SCALING == 'MinMax':
            for i in range(0,2):
                inputs_test[:,:,i] = inputs_test[:,:,i] - minmax_in[2*i] / (minmax_in[2*i+1]-minmax_in[2*i])
        if SCALING == 'Std':
            inputs_test[:,:] = (inputs_test[:,:]-mean_in)/std_in
        
    # Convert to tensor
    X = torch.tensor(inputs_test[:,:,:], dtype=torch.float32)
    Y = torch.tensor(outputs_test[:,:,:], dtype=torch.float32)
    dataset = []
    for i in range(len(X)):
        dataset.append((X[i],Y[i]))
   
    val_loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=False, drop_last=True)
    
    return val_loader
#############################################################################################

def train_dataloader(inputs, outputs, index_end):
    inputs_copy = inputs.copy()
    outputs_copy = outputs.copy()
    
    inputs_train = np.float64(inputs_copy[:index_end,:,:])
    outputs_train = np.float64(outputs_copy[:index_end,:,:])
    
    # Input min max scaling
    minmax_in = []
    
    min_in = np.min(inputs_train,(0,1))
    max_in = np.max(inputs_train,(0,1))
    
    std_in = np.std(inputs_train,(0,1))
    mean_in = np.mean(inputs_train,(0,1))
    
    # Outputs standardization
    std_out = np.std(outputs_train,(0,1))
    mean_out = np.mean(outputs_train,(0,1))
    
    min_out = np.min(outputs_train, (0, 1))
    max_out = np.max(outputs_train, (0, 1))
    
    outputs_train[:,:] = (outputs_train[:,:]-mean_out)/std_out
    # outputs_train[:,:] = (outputs_train[:,:] - min_out) / (max_out - min_out)
    
    if MODE == "QuadTubes":
        # Inputs scaling
        if SCALING == 'MinMax':
            for i in range(0,3):
                minmax_in.append(Vmin)
                minmax_in.append(Vmax)
                inputs_train[:,:,i] = inputs_train[:,:,i] - minmax_in[2*i] / (minmax_in[2*i+1]-minmax_in[2*i])
            minmax_in.append(Wmin)
            minmax_in.append(Wmax)
            inputs_train[:,:,3] = inputs_train[:,:,3] - minmax_in[2*3] / (minmax_in[2*3+1]-minmax_in[2*3])
            for i in range(4,7):
                minmax_in.append(Amin)
                minmax_in.append(Amax)
                inputs_train[:,:,i] = inputs_train[:,:,i] - minmax_in[2*i] / (minmax_in[2*i+1]-minmax_in[2*i])
        
        if SCALING == 'Std':
            inputs_train[:,:] = (inputs_train[:,:]-mean_in)/std_in
        
    elif MODE == "UnicTubes":
        # Inputs scaling
        if SCALING == 'MinMax':
            for i in range(0,2):
                minmax_in.append(min_in[i])
                minmax_in.append(max_in[i])
                inputs_train[:,:,i] = inputs_train[:,:,i] - minmax_in[2*i] / (minmax_in[2*i+1]-minmax_in[2*i])
        if SCALING == 'Std':
            inputs_train[:,:] = (inputs_train[:,:]-mean_in)/std_in
        
    # Convert to tensor
    X = torch.tensor(inputs_train[:,:,:], dtype=torch.float32)
    Y = torch.tensor(outputs_train[:,:,:], dtype=torch.float32)
    dataset = []
    for i in range(len(X)):
        dataset.append((X[i],Y[i]))
    
    train_loader = DataLoader(dataset, batch_size=128, shuffle=False, drop_last=True)
    return train_loader, std_out, mean_out, minmax_in, std_in, mean_in, min_out, max_out, min_in, max_in
#############################################################################################

if __name__ == "__main__":
    #Load the datasets
    # Construct the relative path to the desired file inside CAMP
    # in_train = os.path.join('src', 'results', 'Unicycle', 'Learning', 'No_Acc', '10k_15m_Tl1m', 'input.npy')
    # out_train = os.path.join('src', 'results', 'Unicycle', 'Learning', 'No_Acc', '10k_15m_Tl1m', 'output.npy')
    # in_test = os.path.join('src', 'results', 'Unicycle', 'Learning', 'No_Acc', '1k_15m_Tl1_1m', 'input.npy')
    # out_test = os.path.join('src', 'results', 'Unicycle', 'Learning', 'No_Acc', '1k_15m_Tl1_1m', 'output.npy')
    
    in_train = os.path.join('src', 'results', 'Quad_Learning', '10K_15s_tl1_RK2', 'input.npy')
    out_train = os.path.join('src', 'results', 'Quad_Learning', '10K_15s_tl1_RK2', 'output.npy')
    in_test = os.path.join('src', 'results', 'Quad_Learning', '10K_15s_tl1_RK2', 'input.npy')
    out_test = os.path.join('src', 'results', 'Quad_Learning', '10K_15s_tl1_RK2', 'output.npy')
    
    path_in_train = os.path.join(camp_dir, in_train)
    path_out_train = os.path.join(camp_dir, out_train)
    path_in_test = os.path.join(camp_dir, in_test)
    path_out_test = os.path.join(camp_dir, out_test)
    
    inputs_train = np.load(path_in_train)
    outputs_train = np.load(path_out_train)
    inputs_test = np.load(path_in_test)
    outputs_test = np.load(path_out_test)
    
    index_split = int(len(inputs_train)*0.8) #To split the main dataset in the training set and validation set
    
    data, std_out, mean_out, minmax_in, std_in, mean_in, min_out, max_out, min_in, max_in = train_dataloader(inputs_train, outputs_train, index_split) # Set used for the trining, used to recover the mean and std of outputs components
    
    if VAL:
        val = val_dataloader(inputs_train, outputs_train, std_out, mean_out, minmax_in, std_in, mean_in, min_out, max_out, min_in, max_in, index_split) # Validation set
    else:
        val = val_dataloader(inputs_test[:], outputs_test[:], std_out, mean_out, minmax_in, std_in, mean_in, min_out, max_out, min_in, max_in, 0) # Test set
    print("Data succesfully loaded !")
    
    if LSTM:
        HIDDEN = 512 
        modelLight = LightningNetLSTM.load_from_checkpoint("YOUR MODEL PATH", HIDDEN=HIDDEN)
    elif RNN:
        HIDDEN = 512 
        modelLight = LightningNetRNN.load_from_checkpoint("YOUR MODEL PATH", HIDDEN=HIDDEN)
    elif GRU:
        if MODE == "QuadTubes":
            HIDDEN = 512 
            model_relative_path = os.path.join('src', 'learning_sensitivity', 'models', 'GRU_logs', 'lightning_logs', 'version_0', 'checkpoints', 'epoch=99-step=99.ckpt')
            model_path = os.path.join(camp_dir, model_relative_path)
            modelLight = LightningNetGRU.load_from_checkpoint(model_path, HIDDEN=HIDDEN)
        elif MODE == "UnicTubes":
            HIDDEN = 64 
            model_relative_path = os.path.join('src', 'learning_sensitivity', 'models', 'GRU_logs', 'lightning_logs', 'unicycle_no_acc', 'checkpoints', 'epoch=49-step=49.ckpt')
            model_path = os.path.join(camp_dir, model_relative_path)
            modelLight = LightningNetGRU.load_from_checkpoint(model_path, HIDDEN=HIDDEN)
    elif MLP:
        HIDDEN = 512
        modelLight = LightningNetMLP.load_from_checkpoint("YOUR MODEL PATH", HIDDEN=HIDDEN)
    elif TRANS:
        HIDDEN = 512
        D_MODEL = 512
        NHEAD = 1
        N_LINEAR_LAYERS = 3
        N_TRANS_LAYERS = 1
        model_relative_path = os.path.join('src', 'learning_sensitivity', 'models', 'Transformer_logs', 'lightning_logs', '4heads', 'checkpoints', 'epoch=171-step=171.ckpt')
        model_path = os.path.join(camp_dir, model_relative_path)
        modelLight = LightningNetTransformer.load_from_checkpoint(model_path)
    modelLight.eval()
    print(modelLight.model)
    
    #Save model to .pt format
    m = modelLight.model.to(device)
    data = np.random.normal(1, 1, size=(1, 250, INPUT_SIZE))
    data = torch.Tensor(data).to(device)
    h0, c0 = torch.zeros(1, 1, HIDDEN).to(device), torch.zeros(1, 1, HIDDEN).to(device)

    if LSTM:
        traced_script_module = torch.jit.trace(m, (data, h0,c0))
    elif RNN or GRU:
        traced_script_module = torch.jit.trace(m, (data, h0)) 
    elif MLP or TRANS: 
        traced_script_module = torch.jit.trace(m, data)    
    traced_script_module.save('sensitivityNN.pt')
    
    # Test pred 
    with torch.no_grad():
        
        # Trajectory of n states filled of 0
        nb_state = 100
        test_traj = torch.zeros(1,nb_state,INPUT_SIZE)
        h0, c0 = torch.zeros(1, 1, HIDDEN).to(device), torch.zeros(1, 1, HIDDEN).to(device)
        
        # Compute the mean inference time for 100 predictions
        test_time = []
        for k in range(0,1000):
            tstart_pred = time.time()
            if LSTM:
                pred, (h0, c0) = modelLight.model(test_traj, h0, c0)
            elif RNN or GRU:
                pred, h0 = modelLight.model(test_traj, h0)
            elif MLP or TRANS:
                pred = modelLight.model(test_traj)
            tend_pred = time.time()
            test_time.append(tend_pred-tstart_pred)
        print("Time prediction average: ", np.mean(test_time))
        print("Time prediction std: ", np.std(test_time))
        
        for batch in val:
            (X,y) = batch
            
            h0 = torch.zeros(1,BATCH_SIZE,HIDDEN).to(device)
            c0 = torch.zeros(1,BATCH_SIZE,HIDDEN).to(device)
            
            if LSTM:
                pred, (h0, c0) = modelLight.model(X[:,:,:],h0,c0)
            elif RNN or GRU:
                pred, h0 = modelLight.model(X[:,:,:],h0)
            elif MLP or TRANS:
                pred = modelLight.model(X[:,:,:])

    result = (pred.to('cpu').numpy())*std_out+mean_out
    y_norm = (y[:,:,:].numpy())*std_out+mean_out
    result_conc = np.concatenate(result[:,4:,:], 0)
    y_norm_conc = np.concatenate(y_norm[:,4:,:], 0)
    
    import sklearn.metrics as sm
    if MODE == "QuadTubes":
        mae_rq = sm.mean_absolute_error(np.linalg.norm(y_norm_conc[:,0:3], axis = 1), np.linalg.norm(result_conc[:,0:3], axis = 1))
        mae_u = sm.mean_absolute_error(np.linalg.norm(y_norm_conc[:,3:7], axis = 1), np.linalg.norm(result_conc[:,3:7], axis = 1))
        mae_ru = sm.mean_absolute_error(np.linalg.norm(y_norm_conc[:,7:11], axis = 1), np.linalg.norm(result_conc[:,7:11], axis = 1))
    elif MODE == "UnicTubes":
        mae_rq = sm.mean_absolute_error(np.linalg.norm(y_norm_conc[:,0:2], axis = 1), np.linalg.norm(result_conc[:,0:2], axis = 1))
        mae_u = sm.mean_absolute_error(np.linalg.norm(y_norm_conc[:,2:4], axis = 1), np.linalg.norm(result_conc[:,2:4], axis = 1))
        mae_ru = sm.mean_absolute_error(np.linalg.norm(y_norm_conc[:,4:6], axis = 1), np.linalg.norm(result_conc[:,4:6], axis = 1))
        
    print('MAE Rq: ', mae_rq)
    print('MAE U: ', mae_u)
    print('MAE Ru: ', mae_ru)
    
    
    from random import randint 
    index = randint(0, 500) # Get a random trajectory from the test or validation set
    # index = 337 # Display the trajectory number 103
    lenght_traj = 300 # number of states to plot
    
    if MODE == "QuadTubes":
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(np.linalg.norm(y_norm[index,:lenght_traj,0:3], axis = 1), label='ground truth', linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(np.linalg.norm(result[index,:lenght_traj,0:3], axis = 1), label='GRU prediction', linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('rq',fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.legend()
        plt.grid()
        plt.subplot(3,1,2)
        plt.plot(np.linalg.norm(y_norm[index,:lenght_traj,3:7], axis = 1), linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(np.linalg.norm(result[index,:lenght_traj,3:7], axis = 1), linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('u', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.subplot(3,1,3)
        plt.plot(np.linalg.norm(y_norm[index,:lenght_traj,7:], axis = 1), linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(np.linalg.norm(result[index,:lenght_traj,7:], axis = 1), linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ru', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(y_norm[index,:,0], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,0], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('rx',fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.legend()
        plt.subplot(3,1,2)
        plt.plot(y_norm[index,:,1], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,1], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ry', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.subplot(3,1,3)
        plt.plot(y_norm[index,:,2], linewidth=5.5, label='ground truth', linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,2], linewidth=3.5, label='GRU prediction', color = 'deepskyblue')
        plt.ylabel('rz', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.legend()
        plt.grid()
        
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(y_norm[index,:,3], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,3], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('u1', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.legend()
        plt.subplot(3,1,2)
        plt.plot(y_norm[index,:,4], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,4], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('u2', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.subplot(3,1,3)
        plt.plot(y_norm[index,:,5], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,5], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('u3', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(y_norm[index,:,6], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,6], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('u4', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.legend()
        plt.subplot(3,1,2)
        plt.plot(y_norm[index,:,7], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,7], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ru1', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.subplot(3,1,3)
        plt.plot(y_norm[index,:,8], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,8], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ru2', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(y_norm[index,:,9], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,9], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ru3', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.legend()
        plt.subplot(3,1,2)
        plt.plot(y_norm[index,:,10], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,10], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ru4', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()

        plt.show()
        
        # Compute the quantile for each component to be exported
        ecarts_relatif = abs(result_conc - y_norm_conc)
                
        d_quantile = {'Rx': ecarts_relatif[:,0], 'Ry': ecarts_relatif[:,1], 'Rz': ecarts_relatif[:,2], 'u1': ecarts_relatif[:,3], 'u2': ecarts_relatif[:,4], 'u3': ecarts_relatif[:,5], 'u4': ecarts_relatif[:,6], 
                        'Ru1': ecarts_relatif[:,7], 'Ru2': ecarts_relatif[:,8], 'Ru3': ecarts_relatif[:,9], 'Ru4': ecarts_relatif[:,10]}
        df_quantile = pd.DataFrame(data=d_quantile)
        quantile = []
        quantile.append(float(df_quantile.Rx.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.Ry.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.Rz.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.u1.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.u2.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.u3.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.u4.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.Ru1.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.Ru2.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.Ru3.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.Ru4.quantile([0.75]).values[0]))
            
        dyaml = {'std_in_coeff_tubes':std_in.tolist(), 'mean_in_coeff_tubes':mean_in.tolist(), 'std_out_coeff_tubes':std_out.tolist(), 'mean_out_coeff_tubes':mean_out.tolist(), 'min_out_coeff_tubes':min_out.tolist(), 'max_out_coeff_tubes':max_out.tolist(), 'min_in_coeff_tubes':min_in.tolist(), 'max_in_coeff_tubes':max_in.tolist(), 'quartile_out_coeff_tubes':quantile}
        with open('sensiNN_std_coeff.yaml', 'w') as yaml_file:
            yaml.dump(dyaml, yaml_file, default_flow_style=False)
            
    elif MODE == "UnicTubes":
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(np.linalg.norm(y_norm[index,:lenght_traj,0:2], axis = 1), label='ground truth', linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(np.linalg.norm(result[index,:lenght_traj,0:2], axis = 1), label='GRU prediction', linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('rq',fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.legend()
        plt.grid()
        plt.subplot(3,1,2)
        plt.plot(np.linalg.norm(y_norm[index,:lenght_traj,2:4], axis = 1), linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(np.linalg.norm(result[index,:lenght_traj,2:4], axis = 1), linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('u', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.subplot(3,1,3)
        plt.plot(np.linalg.norm(y_norm[index,:lenght_traj,4:], axis = 1), linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(np.linalg.norm(result[index,:lenght_traj,4:], axis = 1), linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ru', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        
        plt.figure()
        plt.subplot(2,1,1)
        plt.plot(y_norm[index,:,0], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,0], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('rx',fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.legend()
        plt.subplot(2,1,2)
        plt.plot(y_norm[index,:,1], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,1], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ry', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.legend()
        plt.grid()
        
        plt.figure()
        plt.subplot(2,1,1)
        plt.plot(y_norm[index,:,2], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,2], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('u1', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.legend()
        plt.subplot(2,1,2)
        plt.plot(y_norm[index,:,3], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,3], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('u2', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        
        plt.figure()
        plt.subplot(2,1,1)
        plt.plot(y_norm[index,:,4], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,4], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ru1', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()
        plt.legend()
        plt.subplot(2,1,2)
        plt.plot(y_norm[index,:,5], linewidth=5.5, linestyle = 'dashed', color = 'black')
        plt.plot(result[index,:,5], linewidth=3.5, color = 'deepskyblue')
        plt.ylabel('ru2', fontsize = 25)
        plt.tick_params(axis='both', which='major', labelsize=35)
        plt.grid()

        plt.show()
        
        # Compute the quantile for each component to be exported
        ecarts_relatif = abs(result_conc - y_norm_conc)
                
        d_quantile = {'Rx': ecarts_relatif[:,0], 'Ry': ecarts_relatif[:,1], 'u1': ecarts_relatif[:,2], 'u2': ecarts_relatif[:,3], 'Ru1': ecarts_relatif[:,4], 'Ru2': ecarts_relatif[:,5]}
        df_quantile = pd.DataFrame(data=d_quantile)
        quantile = []
        quantile.append(float(df_quantile.Rx.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.Ry.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.u1.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.u2.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.Ru1.quantile([0.75]).values[0]))
        quantile.append(float(df_quantile.Ru2.quantile([0.75]).values[0]))
            
        dyaml = {'std_out_coeff_tubes':std_out.tolist(), 'mean_out_coeff_tubes':mean_out.tolist(), 'min_out_coeff_tubes':min_out.tolist(), 'max_out_coeff_tubes':max_out.tolist(), 'quartile_out_coeff_tubes':quantile}
        with open('sensiNN_std_coeff.yaml', 'w') as yaml_file:
            yaml.dump(dyaml, yaml_file, default_flow_style=False)
    
    print("Done")
    