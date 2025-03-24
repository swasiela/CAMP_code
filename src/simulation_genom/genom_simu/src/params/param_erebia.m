function uav = param_erebia()

%% UAV PARAM
uav.name = 'QR_3';  % name for optitrack
uav.calib = '2023_11_29_calibration_erebia_stddev_increased.mat';  % name of calibration file
uav.tty = '/dev/ttyUSB0';  % name of gazebo pty
uav.arduio_tty = '/dev/ttyUSB1';  % name of arduino usb port
uav.host = 'erebia-wifi';  % host of genomix
uav.mocap.host = 'muybridge';  % mocap host
uav.mocap.port = '1510';  % mocap port

% ---------------------------------------------------------- { Env. Vars. }
uav.openrobots_dir = getenv('ROBOTPKG_BASE');
% Check if $ROBOTPKG_BASE exists
if isempty(uav.openrobots_dir)
    error('Environmental variable $ROBOTPKG_BASE not found!');
else
    disp(['Found $ROBOTPKG_BASE: ', uav.openrobots_dir]);
end

uav.rpath = {'/lib/genom/pocolibs/plugins'};  % genom devel path

% Get path relative to the CAMP directory
currentDir = fileparts(mfilename('fullpath'));

% Navigate up three levels to reach the CAMP folder
campDir = fullfile(currentDir, '..', '..', '..','..','..');  % Adjust based on your structure
% Resolve the CAMP directory path to its absolute form
campDir = char(java.io.File(campDir).getCanonicalPath());

% Construct the path to the results directory relative to the CAMP folder
resultsPath = fullfile(campDir, 'src', 'results');

uav.logs.dir = resultsPath;  % log folder location
uav.logs.traj_dir = resultsPath; 

uav.logs.rate = 1;  % log decimation rate


%% GENOM COMPONENTS
uav.g.opti = 'optitrack';
uav.g.rc = 'rotorcraft';
uav.g.pom = 'pom';
uav.g.man = 'maneuver';
uav.g.nhfc = 'nhfc';
uav.g.mrsim = 'mrsim';
uav.g.arduio = 'arduio';


%% INERTIAL PARAM
dm = 0.00; %offset to make the altitude of the drone match the simulation
uav.inertia.g = 9.81;
uav.inertia.m = 1.098 + dm; % 1.098 (classic) 1.141 (perche) 1.465 (with magnet & package)
uav.inertia.J = num2cell(reshape(diag([0.015, 0.015, 0.007]),[9,1]));


%% PROP PARAMS
uav.props.n = 4;
uav.props.c_f = 5.9e-4; 
uav.props.c_t = 1e-5;

uav.wmin = 16;  % min prop rotation speed
uav.wmax = 110;  % max prop rotation speed


%% ROTORCRAFT FILTERS (for paparazzi, set to 0 for mk or simu)
uav.filters.gfc = {0; 0; 0};
uav.filters.afc = {0; 0; 0};
uav.filters.mfc = {0; 0; 0};


%% ALLOCATION
uav.gx = 0.0; % 0 (window) 0.018 (rings)
uav.gy = 0.0; % 0 (window) 0.009 (rings)
uav.gz = 0.0;
uav.d = 0.23;  % arm length
uav.mbodyw = 0.11;
uav.mbodyh = 0.17;
uav.mmotor = 0.07;
uav.rx = 0;
uav.ry = 0;
uav.rz = -1;

d = uav.d;
c = uav.props.c_t/uav.props.c_f;
% G = [     0,      0,     0,      0  0, 0, 0, 0; ...
%           0,      0,     0,      0  0, 0, 0, 0; ...
%           1,      1,     1,      1, 0, 0, 0, 0; ...
%           -uav.gy,      d-uav.gy,     -uav.gy,     -d-uav.gy, 0, 0, 0, 0; ...
%          uav.gx-d,      uav.gx,     d+uav.gx,      uav.gx, 0, 0, 0, 0; ...
%           c,     -c,     c,     -c, 0, 0, 0, 0 ];
G = [     0,      0,     0,      0  0, 0, 0, 0; ...
          0,      0,     0,      0  0, 0, 0, 0; ...
          1,      1,     1,      1, 0, 0, 0, 0; ...
          0,      d,     0,     -d, 0, 0, 0, 0; ...
         -d,      0,     d,      0, 0, 0, 0, 0; ...
          c,     -c,     c,     -c, 0, 0, 0, 0 ];
uav.G = G*uav.props.c_f;


%% NHFC GAINS

filename = fullfile(resultsPath, 'default_gains.txt');
% filename = fullfile(resultsPath, 'gains_file.txt');
% READING GAIN FILE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kx = zeros(1,200);
kv = zeros(1,200);
ki = zeros(1,200);
kR = zeros(1,200);
kOmega = zeros(1,200);

reading_kX = false;
reading_kV = false;
reading_kI = false;
reading_kR = false;
reading_kOmega = false;
reading_cost = false;

compt_kx = 1;
compt_kv = 1;
compt_ki = 1;
compt_kR = 1;
compt_kOmega = 1;

fid = fopen(filename,'r');
while ~feof(fid)
    tline = fgetl(fid);
    if contains(tline, 'Gains')
        kx = zeros(1,200);
        kv = zeros(1,200);
        ki = zeros(1,200);
        kR = zeros(1,200);
        kOmega = zeros(1,200);
        compt_kx = 1;
        compt_kv = 1;
        compt_ki = 1;
        compt_kR = 1;
        compt_kOmega = 1;
        continue
    end
    if contains(tline, 'Vector number : 0')
        reading_kX = true;
        reading_kV = false;
        reading_kI = false;
        reading_kR = false;
        reading_kOmega = false;
        reading_cost = false;
        continue
    end
    if contains(tline, 'Vector number : 1')
        reading_kX = false;
        reading_kV = true;
        reading_kI = false;
        reading_kR = false;
        reading_kOmega = false;
        reading_cost = false;
        continue
    end
    if contains(tline, 'Vector number : 2')
        reading_kX = false;
        reading_kV = false;
        reading_kI = true;
        reading_kR = false;
        reading_kOmega = false;
        reading_cost = false;
        continue
    end
    if contains(tline, 'Vector number : 3')
        reading_kX = false;
        reading_kV = false;
        reading_kI = false;
        reading_kR = true;
        reading_kOmega = false;
        reading_cost = false;
        continue
    end
    if contains(tline, 'Vector number : 4')
        reading_kX = false;
        reading_kV = false;
        reading_kI = false;
        reading_kR = false;
        reading_kOmega = true;
        reading_cost = false;
        continue
    end
    if contains(tline, 'Cost')
        reading_kX = false;
        reading_kV = false;
        reading_kI = false;
        reading_kR = false;
        reading_kOmega = false;
        reading_cost = true;
        continue
    end
    if reading_cost
        continue
    end
    if reading_kX
        line_comps = split(tline,' ');
        kx(compt_kx) = (str2double(line_comps(1)));
        compt_kx = compt_kx+1;
    end
    if reading_kV
        line_comps = split(tline,' ');
        kv(compt_kv) = (str2double(line_comps(1)));
        compt_kv = compt_kv+1;
    end
    if reading_kI
        line_comps = split(tline,' ');
        ki(compt_ki) = (str2double(line_comps(1)));
        compt_ki = compt_ki+1;
    end
    if reading_kR
        line_comps = split(tline,' ');
        kR(compt_kR) = (str2double(line_comps(1)));
        compt_kR = compt_kR+1;
    end
    if reading_kOmega
        line_comps = split(tline,' ');
        kOmega(compt_kOmega) = (str2double(line_comps(1)));
        compt_kOmega = compt_kOmega+1;
    end
end
fclose(fid);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

uav.nhfc.Kp = [kx(1) kx(3)];
uav.nhfc.Kv = [kv(1) kv(3)];
uav.nhfc.Kq = [kR(1) kR(3)];
uav.nhfc.Kw = [kOmega(1) kOmega(3)];
uav.nhfc.Ki = [ki(1) ki(3)];

uav.uap = {};


%% EMERG PARAM
uav.emerg.e_p_sat = 2.0;
uav.emerg.e_v_sat = 2.0;
uav.emerg.descent = 1.2;
uav.emerg.dx = 0.1;
uav.emerg.dv = 0.3;
uav.emerg.dq = 1;
uav.emerg.dw = 1;

end
