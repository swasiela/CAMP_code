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

disp('*** TEST MAGNET ***')

%% hold

gs.arduio.out1(5,0); 
gs.arduio.out1(6,1); 

disp('HOLD, press ENTER to release');
pause;

%% neutral
gs.arduio.out1(5,0); 
gs.arduio.out1(6,0); 

%% release
gs.arduio.out1(5,1); 
gs.arduio.out1(6,0); 
disp('RELEASED, press ENTER to stop');
pause;

