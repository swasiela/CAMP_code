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
import math

from torch.utils.tensorboard.summary import hparams
from lightning.pytorch import LightningModule

INPUT_SIZE = 7 
OUTPUT_SIZE = 11 

# device = "cpu"
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class PositionalEncoding(nn.Module):

    def __init__(self, d_model, max_len=600):
        """
        Inputs
            d_model - Hidden dimensionality of the input.
            max_len - Maximum length of a sequence to expect.
        """
        super().__init__()

        # Create matrix of [SeqLen, HiddenDim] representing the positional encoding for max_len inputs
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe.unsqueeze(0)

        # register_buffer => Tensor which is not a parameter, but should be part of the modules state.
        # Used for tensors that need to be on the same device as the module.
        # persistent=False tells PyTorch to not add the buffer to the state dict (e.g. when we save the model)
        self.register_buffer('pe', pe, persistent=False)

    def forward(self, x):
        x = x + self.pe[:, :x.size(1)]
        return x

class NetTransformer(nn.Module) :
    def __init__(self, D_MODEL=512, NHEAD=8, N_TRANS_LAYERS=6, N_LINEAR_LAYERS=3):
        super(NetTransformer, self).__init__()
        
        self.layer_in = nn.Linear(INPUT_SIZE, D_MODEL).to(device)
        
        # Positional encoding for sequences
        self.positional_encoding = PositionalEncoding(d_model=D_MODEL)
        
        encoder_layer = nn.TransformerEncoderLayer(d_model=D_MODEL, nhead=NHEAD, batch_first=True).to(device)
        self.transformer_encoder = nn.TransformerEncoder(encoder_layer, num_layers=N_TRANS_LAYERS).to(device)
        
        layers = []
        in_dim = D_MODEL
        for i in range(N_LINEAR_LAYERS-1):
            output_dim = in_dim // 2
            layers.append(nn.Linear(in_dim, output_dim).to(device))
            in_dim = output_dim
        layers.append(nn.Linear(in_dim, OUTPUT_SIZE).to(device))
        self.layers = nn.ModuleList(layers)

    def forward(self, inputs):
        inputs = inputs.to(device)
        # Linear encoding
        x = self.layer_in(inputs)
        
        # Positionnal encoding
        x = self.positional_encoding(x)
        
        # Trans
        x = self.transformer_encoder(x)
        
        # Decoding
        for i in range(len(self.layers)-1):
            x = self.layers[i](x)
            x = torch.relu(x)
        x = self.layers[-1](x)
        
        return x

class LightningNetTransformer(LightningModule):
    def __init__(self, LR=1e-3, D_MODEL=512, NHEAD=8, N_TRANS_LAYERS=6, N_LINEAR_LAYERS=3, std_out=0, mean_out=0):
        super().__init__()
        self.model = NetTransformer(D_MODEL, NHEAD, N_TRANS_LAYERS, N_LINEAR_LAYERS)
        
        self.save_hyperparameters("LR")
        self.save_hyperparameters("D_MODEL")
        self.save_hyperparameters("NHEAD")
        self.save_hyperparameters("N_TRANS_LAYERS")
        self.save_hyperparameters("N_LINEAR_LAYERS")
        
        
        self.criterion = nn.MSELoss()
        self.learning_rate = LR
        
        self.std_out = std_out
        self.mean_out = mean_out

    def forward(self, data):
        return self.model(data)

    def training_step(self, batch, batch_nb):
        data, target = batch
        output = self.forward(data)
        loss = self.criterion(output, target)
        self.log_dict({"train_loss": loss, "step": float(self.current_epoch)}, on_step=False, on_epoch=True, prog_bar=True, logger=True)
        return loss

    def validation_step(self, batch, batch_nb):
        data, target = batch
        output = self.forward(data)
        loss =  self.criterion(output)
        self.logger.log_hyperparams(self.logger.hparams)
        self.log_dict({"val_loss": loss, "step": float(self.current_epoch)}, on_step=False, on_epoch=True, prog_bar=False, logger=True)
        self.logger.log_metrics({"hp_metric" : loss})
        return loss

    def configure_optimizers(self):
        optimizer=optim.Adam(self.model.parameters(), lr = self.learning_rate)
        return optimizer