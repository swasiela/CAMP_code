%% Description
% This script allows you to get the calibration matrix of either the
% Paparazzi or Mikrokopter IMU.
%

clear all;
close all;
clc;

%% USER OPTIONS
usropt.robot_pc_name = 'erebia_calib'; % 'erebia_calib'

usropt.result_dir = 'results'; % directory where the calibration result is saved
usropt.flags.add_date = true; % if true, it pre-appends the current date to the filename
usropt.flags.txt_version = false; % if true, it creates a txt file of the calibration (in <result_dir>/txt)

usropt.data.format = 'yyyy_mm_dd';

usropt.txt.delimiter = ' '; % delimiter between array elements of each calibration field
usropt.txt.cnt_mode = 1; % in txt file, specify array elements' indexes from 0 (1) or from 1 (2)
usropt.txt.style = 'gi';   % 'none': field_name field_value | 'gi':  field_name = {field_value}
usropt.txt.precdigits = 10; % number of digits after comma for each value

%% INIT
addpath(genpath('genom'));
addpath(genpath('pcs'));

pc = eval(['param_' usropt.robot_pc_name]);
g = init_genom_imu(pc);

if usropt.flags.add_date    
    current_date = datestr(now, usropt.data.format);
end

whitespace = 32;
if exist(usropt.result_dir, 'dir') ~= 7 % if result directory does not exist
      mkdir(usropt.result_dir); % create it
end
if usropt.flags.txt_version % if usropt.flags.txt_version is enabled
  txtResultDir = [usropt.result_dir, filesep, 'txt']; % then create txt-result directory name
  if exist(txtResultDir, 'dir') ~= 7 % if that folder does not exist
      mkdir(txtResultDir); % create it
  end
end
input('Press <enter> to continue. ', 's');

%% IMU CALIBRATION

disp(('Zero the accelerometer and gyrometer biases...'));
disp(' Put the aerial platform on the ground or on another flat surface.');
input(' Once you have done it, press ENTER ', 's');
g.fc.set_zero();
disp('done');

disp(('Get IMU calibration...'));
calibration = g.fc.get_imu_calibration().result;
disp('done');

disp(('Save result...'));
if strcmp(usropt.result_dir, '') ~= 1 % If saving directory has been specified
    if exist(usropt.result_dir, 'dir') ~= 7 % if saving directory does not exist
        mkdir(usropt.result_dir); % create it!
    end
    usropt.result_dir = [usropt.result_dir, filesep]; % append filesep
end
filename = input(' Specify here the filename for saving the result: ', 's');
if usropt.flags.add_date
    filename = [current_date, '_', filename];
end
save([usropt.result_dir, filename], 'calibration');
if usropt.flags.txt_version
    structfields2txt(txtResultDir, filename, calibration.imu_calibration, ...
          usropt.txt.delimiter, usropt.txt.cnt_mode, usropt.txt.style, usropt.txt.precdigits);
end
disp([' Calibration result saved as ''', filename, '''']);
disp([' at ''', usropt.result_dir, '''']);
disp('done');

disp(' ');
pause(0.5);
test = input('Would like to test the result? (y/n): ', 's');

disp(('Test'));
if strcmpi(test, 'y')
    while 1
        disp(' Accelerometer:');
        g.fc.imu().imu.acc
        disp('******');

        skip = input(' Press <enter> to get another sample or type <n> for exit: ' , 's');
        if strcmpi(skip, 'n') break; end %#ok<SEPEX>
    end
end
disp('done');

disp(('Set zero completed!'));
