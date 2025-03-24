function genom = init_genom_imu(pc)
  %% GENOMIX g.client
  disp(('Retrieve g.client...'));
  genom.client = genomix.client(pc.hostname); 
  genom.client.rpath([pc.openrobots_dir, '/lib/genom/pocolibs/plugins']);
  genom.client.rpath([pc.devel_dir, '/lib/genom/pocolibs/plugins']);
  disp('done');
  
  %% ROTORCRAFT
  
  switch pc.fc_module
    case 'mk'
      disp(('Initialize mikrokopter...'));
      % Load mikrokopter module
      genom.fc = genom.client.load('mikrokopter');
    case 'rc'
      disp(('Initialize rotorcraft...'));
      % Load mikrokopter module
      genom.fc = genom.client.load('rotorcraft');
    otherwise
      error('invalid ''pc.fc_module''')
  end
  pause(1);
  disp('done');
  
  % Connect to flicht controller
  msg_fc = genom.fc.connect(pc.usb_port, pc.baud_rate);
  if strcmpi ( msg_fc.status, 'error') == 1
      disp(('>>> Flight controller: error when connecting <<<'));
      disp([('> exception: '), msg_fc.exception.ex]);
      disp([('> code: '), num2str(msg_fc.exception.detail.code)]);
      disp([('> what: '), msg_fc.exception.detail.what]);
      error(' ');
  end
  disp([('Connect to flight controller:'), ' ', msg_fc.status]);
  
  if strcmpi(pc.fc_module, 'rc')
    % threshold for still/movement detection during calibration routine
    disp([' setting calibration still-factor threshold: ', num2str(pc.calib_param)]);
    genom.fc.set_calibration_param(pc.calib_param);
    disp('done');
  end

  load(pc.calib);
  result = genom.fc.set_imu_calibration(calibration);
  disp([' Load IMU calibration: ', result.status]);
end
