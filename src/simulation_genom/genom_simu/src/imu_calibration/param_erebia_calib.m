function pc = param_erebia_calib()
  %% CONFIGURATION
  % PC name hosting genom client.
  % Use <localhost> if genon modules are running on your machine
  % Use <robot_pc_name>-wifi if genon modules are running on robot's pc
  pc.hostname = 'erebia-wifi'; % host where genomix and genom3 modules are running
  pc.fc_module = 'rc'; % 'rc': rotorcraft | 'mk': mikrokopter
  pc.openrobots_dir = '/home/swasiela/openrobots';   % Path to openrobots directory in the specified host
  pc.devel_dir = '/home/swasiela/devel';
  pc.usb_port = '/dev/ttyUSB0';
  pc.calib_param = 40; % tolerance factor standstill detector (default: 10 | suggested paparazzi: 10, mk: 30~50)
  pc.baud_rate = 500000;     % usb baud rate (dafault: 500000)
  pc.calib = '/home/swasiela/CAMP/src/genom_simu/src/params/2023_11_29_calibration_erebia_stddev_increased.mat';  % name of calibration file
end
