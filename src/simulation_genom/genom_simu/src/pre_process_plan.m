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

disp('*** LOG THE PLANNED TRAJECTORY ***')
%% Load the trajectory

dt_cc = 0.05;
% INITIALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Get path relative to the CAMP directory
currentDir = fileparts(mfilename('fullpath'));

% Navigate up three levels to reach the CAMP folder
campDir = fullfile(currentDir, '..', '..', '..','..');  % Adjust based on your structure
% Resolve the CAMP directory path to its absolute form
campDir = char(java.io.File(campDir).getCanonicalPath());

% Construct the path to the results directory relative to the CAMP folder
resultsPath = fullfile(campDir, 'src', 'results');

filename = fullfile(resultsPath, 'robust_traj.txt');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% READING TRAJ FILE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X = zeros(1,30000);
Y = zeros(1,30000);
Z = zeros(1,30000);
R = zeros(1,30000);
P = zeros(1,30000);
Yaw = zeros(1,30000);

Vx = zeros(1,30000);
Vy = zeros(1,30000);
Vz = zeros(1,30000);
Wr = zeros(1,30000);
Wp = zeros(1,30000);
Wy = zeros(1,30000);

Ax = zeros(1,30000);
Ay = zeros(1,30000);
Az = zeros(1,30000);
Wrdot = zeros(1,30000);
Wpdot = zeros(1,30000);
Wydot = zeros(1,30000);

Index = zeros(1,30000);

compt_traj = 1;
compt_idx = 1;

readingTraj = false;
readingUncertainty = false;
readingIndex = false;
readingSim = false;

fid = fopen(filename,'r');
while ~feof(fid)
    tline = fgetl(fid);
    if contains(tline, 'Reference')
        readingTraj = true;
        readingUncertainty = false;
        readingIndex = false;
        readingSim = false;
        continue
    end
    if contains(tline, 'tubes')
        readingTraj = false;
        readingUncertainty = true;
        readingIndex = false;
        readingSim = false;
        continue
    end
    if contains(tline, 'Index Waypoints')
        readingTraj = false;
        readingUncertainty = false;
        readingIndex = true;
        readingSim = false;
        continue
    end
    if contains(tline, 'Nominal')
        readingTraj = false;
        readingUncertainty = false;
        readingIndex = false;
        readingSim = true;
        continue
    end
    if readingSim
        continue
    end
    if readingTraj
        line_comps = split(tline,';');
        X(compt_traj) = str2double(line_comps(2));
        Y(compt_traj) = str2double(line_comps(3));
        Z(compt_traj) = str2double(line_comps(4));
        R(compt_traj) = 0.0;
        P(compt_traj) = 0.0;
        Yaw(compt_traj) = str2double(line_comps(5));
        Vx(compt_traj) = str2double(line_comps(6));
        Vy(compt_traj) = str2double(line_comps(7));
        Vz(compt_traj) = str2double(line_comps(8));
        Wy(compt_traj) = str2double(line_comps(9));
        Ax(compt_traj) = str2double(line_comps(10));
        Ay(compt_traj) = str2double(line_comps(11));
        Az(compt_traj) = str2double(line_comps(12));
        compt_traj = compt_traj+1;
    end
    if readingUncertainty
        continue
    end
    if readingIndex
        line_comps = split(tline,';');
        Index(compt_idx) = str2double(line_comps(2));
        compt_idx = compt_idx + 1;
    end
end
fclose(fid);
X = X(1:compt_traj);
Y = Y(1:compt_traj);
Z = Z(1:compt_traj);
R = R(1:compt_traj);
P = P(1:compt_traj);
Yaw = Yaw(1:compt_traj);
Vx = Vx(1:compt_traj);
Vy = Vy(1:compt_traj);
Vz = Vz(1:compt_traj);
Wy = Wy(1:compt_traj);
Ax = Ax(1:compt_traj);
Ay = Ay(1:compt_traj);
Az = Az(1:compt_traj);
Index = Index(1:compt_idx-1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Trajectory loaded !')

%% WRITE LOG FILE
fileID = fopen(fullfile(resultsPath, 'trajectory.log'),'w');
fprintf(fileID,'%6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s\n','ts','x','y','z','roll','pitch','yaw','vx','vy','vz','wx','wy','wz','ax','ay','az','dwx','dwy','dwz');
for i = 2:+1:compt_traj-1
    fprintf(fileID,'%10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f  %10.7f\n',[(i-1)*dt_cc; X(i); Y(i); Z(i); 0; 0; Yaw(i)-0.0; Vx(i); Vy(i); Vz(i); 0; 0; 0; Ax(i); Ay(i); Az(i); 0; 0; 0]);
end
fclose(fileID);
disp('Trajectory logged !')
