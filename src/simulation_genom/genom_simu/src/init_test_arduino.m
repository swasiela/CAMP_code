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

function genom = init_test_arduino(uav, param)

disp(' ');
disp(['*** NEW UAV - ', uav.name, ' ***']);


%% GENOMIX CLIENT
uav.client = genomix.client(uav.host);
if param.msg == 0
    uav.client.rpath(strcat(cell2mat(uav.rpath), '/lib/genom/pocolibs/plugins/'));
else
    uav.client.rpath(strcat(cell2mat(uav.rpath), '/lib/genom/ros/plugins/'));
end

%% ROTORCRAFT
disp('* Initialize Rotorcraft');
genom.rc = uav.client.load('rotorcraft', '-i', uav.g.rc);

result = genom.rc.connect(uav.tty, 500000);
disp([' Connecting to TTY: ', result.status]);

%% ARDUINO AND ELECTROMAGNET
if param.magnet == 1
    genom.arduio = uav.client.load('arduio', '-i', uav.g.arduio);
    result = genom.arduio.connect(uav.arduio_tty);
    disp([' Connecting to arduino: ', result.status]);
    % Electromagnet control
    genom.arduio.set_dir(5,'::arduio::IO_OUT');
    genom.arduio.set_dir(6,'::arduio::IO_OUT');

    genom.arduio.out1(5,0);
    genom.arduio.out1(6,0);
    disp('Arduino port set');
end