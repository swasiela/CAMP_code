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

%%%%%%%%%%%%%%%%%%%%% SIMPLE_MOTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Defines the experiment (real or simulated) to be performed by the aerial
% platform.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc

log_exec = true;

disp('*** EXECUTE THE PLANNED TRAJECTORY ***')

%% CALIBRATION
% disp('*** setting zeros ***')
% gs.rc.set_zero(10);
% gs.rc.set_zero_velocity(10);

%% INITIALIAZE
gs.rc.set_ramp(2);

xmin = -3.0;
xmax = 3.0;
ymin = -2.5;
ymax = 2.5;
zmin = 0.0;
zmax = 2.0;
yawmin = -3.1415;
yawmax = 3.1415;

vmax = 5.0;
amax = 1.5;
jmax = 15.0;
smax = 30.0;

wmax = 0.5;
dwmax = 0.25;
ddwmax = 8.0;
dddwmax = 16.0;

gs.man.set_bounds(xmin,xmax,ymin,ymax,zmin,zmax,yawmin,yawmax);
gs.man.set_velocity_limit(vmax,wmax);
gs.man.set_acceleration_limit(amax,dwmax);
gs.man.set_jerk_limit(jmax,ddwmax);
gs.man.set_snap_limit(smax,dddwmax);

%% hold magnet
if param.magnet == 1
    gs.arduio.out1(5,0); 
    gs.arduio.out1(6,1); 
end

%% START
disp('rotorcraft.start()');
disp('PAUSE - Press ENTER');
pause;
% Spin propellers at the lowest velocity
gs.rc.start('-a');

%% TAKE OFF
disp('PAUSE - Press ENTER to take off');
pause;

% Set initial planning position to current one
gs.man.set_current_state();

% Track a desired position
gs.nhfc.servo('-a');
gs.rc.servo('-a');

% Vertical take-off from current state
height = 0.6;
gs.man.take_off(height, 5); % activity, height, duration (0: min time)

%% Send to initial state
disp('PAUSE - Press ENTER to send drone to initial state');
pause;
getRobotState;
gs.man.waypoint(-2.0, -1.0, 1.5,0,0, 0, 0, 0, 0, 0, 0, 0);

%% Execute the trajectory
disp('PAUSE - Press ENTER to execute');
pause;

file_log = fullfile(uav.logs.traj_dir, 'trajectory.log');
getRobotState;

if log_exec
    gs.man.log(fullfile(uav.logs.dir, 'man.log'), 1);
    gs.pom.log_state(fullfile(uav.logs.dir, 'pom.log'), 5);
    gs.pom.log_measurements(fullfile(uav.logs.dir, 'pom_meas.log'));
    gs.nhfc.log(fullfile(uav.logs.dir, 'nhfc.log'), 5);
    gs.rc.log(fullfile(uav.logs.dir, 'rc.log'), 5);
end

gs.man.replay(file_log);

%% neutral magnet
if param.magnet == 1
    gs.arduio.out1(5,0); 
    gs.arduio.out1(6,0); 
end

%% release magnet
if param.magnet == 1
    gs.arduio.out1(5,1); 
    gs.arduio.out1(6,0); 
    disp('RELEASED !');
end

gs.man.stop();
% Stop the log
if log_exec
    gs.man.log_stop();
    gs.pom.log_stop();
    gs.nhfc.log_stop();
    gs.rc.log_stop();
end

disp('Execution done !');

%% Land
disp('PAUSE - Press ENTER to land');
pause;
% Vertical land from current state
gs.man.take_off('-a', .35, 0);

%% STOP
disp('rotorcraft.stop()');
disp('PAUSE - Press ENTER');
pause;

% Stop all propellers
gs.rc.stop();
% Stop tracking a desired position
gs.nhfc.stop();
% Cancel any motion and bring the velocity to zero
gs.man.stop();