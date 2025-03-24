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
% In this tutorial, the experiment consists of a sequence of 5 waypoints
% (WP) that the aerial platform (a quadrotor) has to follow.
% The quadrotor starts on the ground and, after starting the propellers, it
% takes off. It follows 5 WPs and then it is commanded to land.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc

disp('*** SIMPLE WAYPOINT EXPERIMENT ***')

%% INITIALIAZE
gs.rc.set_ramp(2);

xmin = -3.0;
xmax = 3.0;
ymin = -1.5;
ymax = 1.5;
zmin = 0.0;
zmax = 2.5;
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
gs.man.take_off('-a', height, 5); % activity, height, duration (0: min time)

gs.man.log(append(uav.logs.dir,'man.log'), 1);
gs.pom.log_state(append(uav.logs.dir,'pom.log'), 5);
gs.pom.log_measurements(append(uav.logs.dir,'pom_meas.log'));
gs.nhfc.log(append(uav.logs.dir,'nhfc.log'), 5);
gs.rc.log(append(uav.logs.dir,'rc.log'), 5);

%% WP1 - Go higher
getRobotState;
x_0 = pos(1);
y_0 = pos(2);
z_0 = pos(3);

if att(3) >= -pi/2 && att(3) <= pi/2
    yaw_des = 0;
else
    yaw_des = pi*sign(att(3));
end

disp('WP1');
disp('PAUSE - Press ENTER');
pause;
% Push a given position to reach after last one
wp1_height = height + 0.5;
gs.man.waypoint(x_0, y_0, wp1_height, yaw_des, 0, 0, 0, 0, 0, 0, 0, 0);
%gs.man.log('CAMP/src/Results/test_log.log', 2);

%% WP2 - Move forward along x axis
getRobotState;

disp('WP2 - Move forward along x axis');
disp('PAUSE - Press ENTER');
pause;
% Push a given position to reach after last one
wp2_x = x_0 + 0.3;
gs.man.waypoint(wp2_x, y_0, wp1_height, yaw_des, 0, 0, 0, 0, 0, 0, 0, 0);

%% WP3 - Rotate by 90 degrees (positive yaw rotation)
getRobotState;

disp('WP3 - Rotate by 90 degrees (positive yaw rotation)');
disp('PAUSE - Press ENTER');
pause;
% Push a given position to reach after last one
yaw_wp3 = yaw_des + pi/2;
gs.man.waypoint(wp2_x, y_0, wp1_height, yaw_wp3, 0, 0, 0, 0, 0, 0, 0, 0);

%% WP4 - Move backward along x axis
getRobotState;

disp('WP4 - Move backward along x axis');
disp('PAUSE - Press ENTER');
pause;
gs.man.waypoint(x_0, y_0, wp1_height, yaw_wp3, 0, 0, 0, 0, 0, 0, 0, 0);

%% WP5 - Rotate by 90 degrees (negative yaw rotation)
getRobotState;

disp('WP5 - Rotate by 90 degrees (negative yaw rotation)');
disp('PAUSE - Press ENTER');
pause;
gs.man.waypoint(x_0,y_0,wp1_height,yaw_des, 0, 0, 0, 0, 0, 0, 0, 0);

gs.man.log_stop();
gs.pom.log_stop();
gs.nhfc.log_stop();
gs.rc.log_stop();

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