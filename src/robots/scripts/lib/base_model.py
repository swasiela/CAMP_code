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

from abc import ABCMeta, abstractmethod
from typing import Optional
from enum import Enum
import numpy as np

from lib.sym_gen import JitParam

class Mode(str, Enum):
    SIMU = "simu"
    GRAD = "grad"
    NOGRAD = "nograd"

class Model(metaclass=ABCMeta):
    N_ctrl_points: int
    ODE: Optional[JitParam]

    def __init__(self, N_ctrl_points):
        super().__init__()
        self.N_ctrl_points = N_ctrl_points
        self._problem = None
        self.ODE = None

    @abstractmethod
    def generate(self, N_lc, verbose=False, mode="nograd", token="", overwrite=False) -> (JitParam) :
        pass
    
    @abstractmethod
    def integrate_along_trajectory(self, des_traj, time_vec, dt):
        pass
    
    @abstractmethod
    def integrate_along_flatten_trajectory(self, flatten_des_traj, dt):
        pass
    
    @abstractmethod
    def optimize_trajectory(self, req):
        pass

    @abstractmethod
    def optimize_mpc(self, req):
        pass
