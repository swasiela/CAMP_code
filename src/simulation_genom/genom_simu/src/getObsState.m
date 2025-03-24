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

%% Retrieve current position and orientation from POM

body = gs.opti.bodies('CAMP_obs1').bodies;
pos = [body.pos.x body.pos.y body.pos.z];
att = [body.att.qw body.att.qx body.att.qy body.att.qz];

% convert quaternions to euler angles (RPY)
eta = att(1);
ex = att(2);
ey = att(3);
ez = att(4);
R = [  (2 * (eta^2 + ex^2) - 1)   (2 * (ex * ey - eta * ez))  (2 * (ex * ez + eta * ey));
           (2 * (ex * ey + eta * ez))   (2 * (eta^2 + ey^2) - 1)   (2 * (ey * ez - eta * ex));
           (2 * (ex * ez - eta * ey))  (2 * (ey * ez + eta * ex))   (2 * (eta^2 + ez^2) - 1)  ];

roll = atan2(R(3, 2), R(3, 3));
pitch = atan2(-R(3, 1), sqrt(R(3, 2)^2 + R(3, 3)^2));
yaw = atan2(R(2, 1), R(1, 1));

att = [roll pitch yaw];

fprintf('Obs1 pos: [%s]\n', join(string(pos), ','));
fprintf('Obs1 ori: [%s]\n', join(string(att), ','));

body = gs.opti.bodies('CAMP_obs2').bodies;
pos = [body.pos.x body.pos.y body.pos.z];
att = [body.att.qw body.att.qx body.att.qy body.att.qz];

% convert quaternions to euler angles (RPY)
eta = att(1);
ex = att(2);
ey = att(3);
ez = att(4);
R = [  (2 * (eta^2 + ex^2) - 1)   (2 * (ex * ey - eta * ez))  (2 * (ex * ez + eta * ey));
           (2 * (ex * ey + eta * ez))   (2 * (eta^2 + ey^2) - 1)   (2 * (ey * ez - eta * ex));
           (2 * (ex * ez - eta * ey))  (2 * (ey * ez + eta * ex))   (2 * (eta^2 + ez^2) - 1)  ];

roll = atan2(R(3, 2), R(3, 3));
pitch = atan2(-R(3, 1), sqrt(R(3, 2)^2 + R(3, 3)^2));
yaw = atan2(R(2, 1), R(1, 1));

att = [roll pitch yaw];

fprintf('Obs2 pos: [%s]\n', join(string(pos), ','));
fprintf('Obs2 ori: [%s]\n', join(string(att), ','));

