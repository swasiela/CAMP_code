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

function genom = init_genom(uav, param)

disp(' ');
disp(['*** NEW UAV - ', uav.name, ' ***']);


%% GENOMIX CLIENT
uav.client = genomix.client(uav.host);
if param.msg == 0
    uav.client.rpath(strcat(cell2mat(uav.rpath), '/lib/genom/pocolibs/plugins/'));
else
    uav.client.rpath(strcat(cell2mat(uav.rpath), '/lib/genom/ros/plugins/'));
end


%% MRSIM
if param.simulator == 0
    disp('* Initialize MRSIM');
    genom.mrsim = uav.client.load('mrsim', '-i', uav.g.mrsim);
    motors = genom.mrsim.get_motor(); motors = motors.result;
    genom.mrsim.set_motor(uav.props.c_f, uav.props.c_t, motors.b, motors.J, motors.K, motors.R, motors.L, motors.V);
    genom.mrsim.set_rotors(uav.props.n);
    genom.mrsim.set_mass(uav.inertia.m);
    genom.mrsim.set_geom(num2cell(reshape(uav.G(:,1:4)',[6*4,1])), uav.inertia.J);
    genom.mrsim.set_state(0,0,0, 0,0,0, 0,0,0, 0,0,0);
    genom.mrsim.set_pty(uav.tty);
end


%% OPTITRACK
if param.simulator == 1
    disp('* Initialize Optitrack');
    genom.opti = uav.client.load('optitrack', '-i', uav.g.opti);
    result = genom.opti.connect(uav.mocap.host, uav.mocap.port, '239.192.168.30', '1511');
    disp([' Connecting to MoCap system: ', result.status]);
end


%% ROTORCRAFT
disp('* Initialize Rotorcraft');
genom.rc = uav.client.load('rotorcraft', '-i', uav.g.rc);

result = genom.rc.connect(uav.tty, 500000);
disp([' Connecting to TTY: ', result.status]);

load(uav.calib);
result = genom.rc.set_imu_calibration(calibration);
disp([' Load IMU calibration: ', result.status]);

result = genom.rc.set_imu_filter(uav.filters.gfc, uav.filters.afc, uav.filters.mfc);
disp([' Set IMU filters: ', result.status]);

result = genom.rc.set_sensor_rate(1000, 0, param.motor_fq, 1); % IMU - mag - motor - battery
disp([' Set sensor rates: ', result.status]);

% ------------------------------------------- { Connect Rotorcraft 2 Nhfc }
result_rc_nhfc = genom.rc.connect_port('rotor_input' , 'nhfc/rotor_input');
% Check that connection went fine
result_conn_rc_nhfc = result_rc_nhfc.status;
if strcmpi (result_conn_rc_nhfc, 'error') == 1
    error('Unable to connect Rotorcraft port (rotor_input) to Nhfc port (rotor_input)!');
end
string = ['Connecting Rotorcraft with Nhfc: ', result_conn_rc_nhfc];
disp(string);

%% POM
if isfield(uav.g, 'pom') && param.state.pom
    disp('* Initialize POM');
    genom.pom = uav.client.load('pom', '-i', uav.g.pom);

    result = genom.pom.connect_port('measure/imu', strcat(uav.g.rc, '/imu'));
    genom.pom.add_measurement('imu', 0,0,0, 0,0,0);
    disp([' Connecting to IMU: ', result.status]);

    if param.simulator == 0
        result = genom.pom.connect_port('measure/mocap', strcat(uav.g.mrsim, '/state'));
    elseif param.simulator == 1
        result = genom.pom.connect_port('measure/mocap', strcat(uav.g.opti, '/bodies/', uav.name));
    end
    genom.pom.add_measurement('mocap', 0,0,0, 0,0,0);
    disp([' Connecting to Optitrack: ', result.status]);

    if param.mag
        result = genom.pom.connect_port('measure/mag', strcat(uav.g.rc, '/mag'));
        genom.pom.add_measurement('mag', 0,0,0, 0,0,0);
        genom.pom.set_mag_field(23.816e-6, -0.41e-6, -39.829e-6);
        disp([' Connecting to magnetometer: ', result.status]);
    end

    genom.pom.set_history_length(0.5);
    r = genom.pom.set_prediction_model('::pom::constant_acceleration');
    r = genom.pom.set_process_noise(200,50);

end


%% MANEUVER
if isfield(uav.g, 'man')
    disp('* Initialize Maneuver');
    genom.man = uav.client.load('maneuver', '-i', uav.g.man);

    if param.state.estimator == 1
        result = genom.man.connect_port('state', strcat(uav.g.tagodom, '/pose'));
        disp([' Connecting to Tagodom: ', result.status]);
    else
        result = genom.man.connect_port('state', strcat(uav.g.pom, '/frame/robot'));
        disp([' Connecting to POM: ', result.status]);
    end

    if param.sim
        genom.man.set_bounds(-10,10,-10,10,0,10,-9,9);
    end

end


%% NHFC
if param.control.default == 1 && isfield(uav.g, 'nhfc')
    disp('* Initialize NHFC');
    genom.nhfc = uav.client.load('nhfc', '-i', uav.g.nhfc);

    genom.nhfc.set_mass(uav.inertia.m);
    genom.nhfc.set_geom(uav.inertia.m, num2cell(reshape(uav.G',[6*8,1])), uav.inertia.J);
    % genom.nhfc.set_gtmrp_geom(uav.props.n, uav.gx, uav.gy, uav.gz, uav.d, uav.inertia.m, uav.mbodyw, uav.mbodyh, uav.mmotor, uav.rx, uav.ry, uav.rz, uav.props.c_f, uav.props.c_t);
    genom.nhfc.set_servo_gain(uav.nhfc.Kp(1), uav.nhfc.Kp(2), uav.nhfc.Kq(1), uav.nhfc.Kq(2), uav.nhfc.Kv(1), uav.nhfc.Kv(2), uav.nhfc.Kw(1), uav.nhfc.Kw(2), uav.nhfc.Ki(1), uav.nhfc.Ki(2));
    genom.nhfc.set_emerg(uav.emerg.descent, uav.emerg.dx, uav.emerg.dq, uav.emerg.dv, uav.emerg.dw);
    genom.nhfc.set_wlimit(uav.wmin, uav.wmax);
    genom.nhfc.set_saturation(uav.emerg.e_p_sat,uav.emerg.e_v_sat,0); %set saturation for the controler

    if param.state.estimator == 1
        result = genom.nhfc.connect_port('state', strcat(uav.g.tagodom, '/pose'));
        disp([' Connecting to Tagodom: ', result.status]);
    else
        result = genom.nhfc.connect_port('state', strcat(uav.g.pom, '/frame/robot'));
        disp([' Connecting to POM: ', result.status]);
    end

    result = genom.nhfc.connect_port('reference', strcat(uav.g.man, '/desired'));
    disp([' Connecting to Maneuver: ', result.status]);
end

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