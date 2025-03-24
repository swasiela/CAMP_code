# importing sys
import sys
import os

# Get the current directory of this script file
current_dir = os.path.dirname(__file__)
parent_dir1 = os.path.abspath(os.path.join(current_dir, '..'))
# adding Scripts to the system path
sys.path.insert(0, parent_dir1)

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader
from torch.utils.tensorboard.summary import hparams

from lightning.pytorch import LightningModule
from lightning.pytorch import Callback
from lightning.pytorch import callbacks
from lightning.pytorch.loggers import TensorBoardLogger
from lightning.pytorch.trainer import Trainer
from lightning.pytorch import loggers as pl_loggers

import numpy as np
import shutil

import optuna
from optuna.samplers import TPESampler, GridSampler

from TransformerEncoder import NetTransformer, LightningNetTransformer
from GRU import NetGRU, LightningNetGRU
from LSTM import NetLSTM, LightningNetLSTM
from RNN import NetRNN, LightningNetRNN
from MLP import NetMLP, LightningNetMLP

# Traverse up the directory tree to find the CAMP directory
camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..'))

MODE = 'QuadTubes' # 'QuadTubes' 'UnicTubes'

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

EPOCHS = 100 # Number of epochs for the training
GRID_SEARCH = False # Use for hyper-parameters optimization

# Set to TRUE the model you want to train
GRU = True
LSTM = False
RNN = False
MLP = False
TRANS = False

#############################################################################################
class MetricsCallback(Callback):
    """PyTorch Lightning metric callback."""

    def __init__(self):
        super().__init__()
        self.metrics = []

    def on_validation_end(self, trainer, pl_module):
        self.metrics.append(trainer.callback_metrics)

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
   
    val_loader = DataLoader(dataset, batch_size=64, shuffle=False, drop_last=True)
    
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
    
    # Load dataset
    
    ##########################################################################
    # Load from a folder
    ##########################################################################
    
    # relative_folder = os.path.join('src', 'learning_sensitivity', 'npy_files', '10K')
    # folder = os.path.join(camp_dir, relative_folder)

    # output_dict = {}
    # input_dict = {}
    
    # outputs_list = []
    # inputs_list = []
    # for file_path in (f.path for f in os.scandir(folder) if f.is_file()):
    #     f_basename = Path(file_path).stem
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
    
    # split 80% of the initial dataset as training set and 20% as validation set
    index_end = int(len(inputs)*0.8) 
    
    # Get the training dataset and the mean and std of the components
    data, std_out, mean_out, minmax_in, std_in, mean_in, min_out, max_out, min_in, max_in = train_dataloader(inputs, outputs, index_end)
    
    # Get the validation set
    val = val_dataloader(inputs, outputs, std_out, mean_out, minmax_in, std_in, mean_in, min_out, max_out, min_in, max_in, index_end)
    print("Data succesfully loaded !")
    
    # Define the objective function for optuna
    def objective(trial: optuna.trial.Trial):
        checkpoint_callback = callbacks.ModelCheckpoint(monitor="val_loss",mode='min')
        metrics_callback = MetricsCallback()
              
        if GRU:
            dir_logger = pl_loggers.TensorBoardLogger(save_dir="src/learning_sensitivity/models/GRU_logs/")
            HIDDEN= 512 # 512 64
            LR = 1e-3 # 1e-3 1e-2
            N_LAYERS = 3
            model = LightningNetGRU(LR, HIDDEN, N_LAYERS, std_out, mean_out)
        elif LSTM:
            dir_logger = pl_loggers.TensorBoardLogger(save_dir="src/learning_sensitivity/models/LSTM_logs/")
            HIDDEN= 512
            LR = 1e-3
            N_LAYERS = 3
            model = LightningNetLSTM(LR, HIDDEN, N_LAYERS, std_out, mean_out)
        elif RNN:
            dir_logger = pl_loggers.TensorBoardLogger(save_dir="src/learning_sensitivity/models/RNN_logs/")
            HIDDEN= 512
            LR = 1e-4
            N_LAYERS = 3
            model = LightningNetRNN(LR, HIDDEN, N_LAYERS, std_out, mean_out)
        elif MLP:
            dir_logger = pl_loggers.TensorBoardLogger(save_dir="src/learning_sensitivity/models/MLP_logs/")
            HIDDEN= 512
            LR = 1e-3
            N_LAYERS = 3
            model = LightningNetMLP(LR, HIDDEN, N_LAYERS, std_out, mean_out)
        elif TRANS:
            dir_logger = pl_loggers.TensorBoardLogger(save_dir="src/learning_sensitivity/models/Transformer_logs/")
            LR = 1e-3
            D_MODEL = 512
            NHEAD = 4
            N_TRANS_LAYERS = 1
            N_LINEAR_LAYERS = 3
            model = LightningNetTransformer(LR, D_MODEL, NHEAD, N_TRANS_LAYERS, N_LINEAR_LAYERS, std_out, mean_out)
        else:
            print("Method not implemented !")
        
        trainer = Trainer(
            logger=dir_logger,
            max_epochs=EPOCHS,
            accelerator="cuda" if torch.cuda.is_available() else "cpu",
            callbacks=[checkpoint_callback, metrics_callback],
        )
        
        trainer.fit(model,data,val)
        return metrics_callback.metrics[-1]["val_loss"].item()

    if GRID_SEARCH:   
        search_space = {"hidden": [32, 64, 128, 256, 512], "learning_rate": [1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1], "n_layers": [1, 2, 3]}
        study = optuna.create_study(direction="minimize", sampler=GridSampler(search_space))
        study.optimize(objective)
    else:
        study = optuna.create_study(direction="minimize", sampler=TPESampler())
        study.optimize(objective, n_trials=1)

    print("Number of finished trials: {}".format(len(study.trials)))

    print("Best trial:")
    trial = study.best_trial

    print("  Value: {}".format(trial.value))

    print("  Params: ")
    for key, value in trial.params.items():
        print("    {}: {}".format(key, value))
    shutil.rmtree(MODEL_DIR)