% Software License Agreement (BSD License)
%
% Copyright (c) 2024, LAAS-CNRS
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright
%    notice, this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright
%    notice, this list of conditions and the following disclaimer in the
%    documentation and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

% Author: Simon WASIELA 

clear all;
close all;
clc;

%% MANUAL PARAMETERS
param.platform = {'erebia_simu'};  % 'qr', 'erebia', 'erebia_simu'

param.simulator = 1;  % use mrsim (0) or gazebo (1)
param.control.default = 1;  % default controller - 0: none (mpc), 1: nhfc, 2: uap

param.state.tagodom = 0;  % enable tagodom (1) or not (0)
param.state.pom = 1;  % enable pom (1) or not (0)
param.state.estimator = 0;  % pom (0) or tagodom (1)

param.joystick = 0;  % use joystick (1) or not (0)

%%% Sensor parameters
param.motor_fq = 50;  % frequency of reading motor data [Hz]
param.mag = 0;  % use mag (1) or not (0)

param.msg = 0; % pocolibs (0) or ros (1)

%%% Use the electromagnet
param.magnet = 0; % (0) don't use (1) use

%% UAV PARAMETERS
% Define param.sim according to which platform is used
if startsWith(param.platform(1), 'erebia')
    param.sim = 0;
else
    param.sim = 1;
end

% Compute number of UAVs
param.Na = size(param.platform, 2);

% Load parameters for each UAVs
uavs = [];
gs = [];
addpath('params');
for i = 1:param.Na
    uavs = [uavs, feval(cell2mat(strcat('param_', param.platform(i))))];
    gs = [gs, init_genom(uavs(i), param)];
    % gs = [gs, init_test_arduino(uavs(i), param)];
end

% Convenience shortcut
if param.Na == 1
    g = gs;
    uav = uavs;
end

%% LOCAL GENOMIX CLIENT
try
    % ------------------------------------------------- { Load Genomix client }
    disp('Retrieve Genomix client...');
    client = genomix.client('localhost');
    if param.msg == 0
        client.rpath([uav.openrobots_dir, '/lib/genom/pocolibs/plugins']);
    else
        client.rpath([uav.openrobots_dir, '/lib/genom/ros/plugins']);
    end
catch
    disp('* No local genomix client launched!')
end

%% BATTERY
if ~param.sim
    disp('* Check Battery');
    for i = 1:param.Na
        batteryLevel = gs(i).rc.get_battery();
        disp([' Battery level (', uavs(i).name, '): ', num2str(batteryLevel.result.battery.level)]);
    end
end
