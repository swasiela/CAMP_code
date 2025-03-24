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
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from torch.utils.tensorboard.summary import hparams
from lightning.pytorch import LightningModule

INPUT_SIZE = 7 
OUTPUT_SIZE = 11 
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class NetRNN(nn.Module) :
    def __init__(self, HIDDEN=512, N_LAYERS=3):
        super(NetRNN, self).__init__()
        self.rnn1 = nn.RNN(INPUT_SIZE, HIDDEN, batch_first=True).to(device)
        layers = []
        in_dim = HIDDEN
        for i in range(N_LAYERS-1):
            output_dim = in_dim // 2
            layers.append(nn.Linear(in_dim, output_dim).to(device))
            in_dim = output_dim
        layers.append(nn.Linear(in_dim, OUTPUT_SIZE).to(device))
        self.layers = nn.ModuleList(layers)

    def forward(self, inputs, h):
        inputs = inputs.to(device)
        if h is None: 
            x, hn = self.rnn1(inputs)
        else:
            x, hn = self.rnn1(inputs,h)
        for i in range(len(self.layers)-1):
            x = self.layers[i](x)
            x = torch.relu(x)
        x = self.layers[-1](x)
        return x, hn

class LightningNetRNN(LightningModule):
    def __init__(self, LR=1e-4, HIDDEN=512, N_LAYERS=3, std_out=0, mean_out=0):
        super().__init__()
        self.model = NetRNN(HIDDEN, N_LAYERS)
        
        self.save_hyperparameters("HIDDEN")
        self.save_hyperparameters("LR")
        self.save_hyperparameters("N_LAYERS")
        
        self.criterion = nn.MSELoss()
        self.learning_rate = LR
        
        self.std_out = std_out
        self.mean_out = mean_out

    def forward(self, data, h=None):
        return self.model(data,h)

    def training_step(self, batch, batch_nb):
        data, target = batch
        output = self.forward(data)
        loss = self.criterion(output[0], target)
        self.log_dict({"train_loss": loss, "step": float(self.current_epoch)}, on_step=False, on_epoch=True, prog_bar=True, logger=True)
        return loss
    
    def validation_step(self, batch, batch_nb):
        data, target = batch
        output = self.forward(data)
        loss =  self.criterion(output[0], target)
        self.logger.log_hyperparams(self.logger.hparams)
        self.log_dict({"val_loss": loss, "step": float(self.current_epoch)}, on_step=False, on_epoch=True, prog_bar=False, logger=True)
        self.logger.log_metrics({"hp_metric" : loss})
        return loss

    def configure_optimizers(self):
        optimizer=optim.Adam(self.model.parameters(), lr = self.learning_rate)
        return optimizer