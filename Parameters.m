%% Parameters of the simulink extension to the CarMaker model
% contains the parameters needed to study the influence of using dynamical
% models on the performance of the low level vehicle control algorithms.
%
%
% author: Charles Philippe (charles.philippe@cranfield.ac.uk)
%
%
%
% The root path when executing this script must be the folder it is in (the
% src4sl folder)


clc; clear;

%% Constants

% declaration
global d2r
global r2d
% global m2in
global in2m

% value
d2r = pi / 180;
r2d = 1 / d2r;
in2m = 0.0254; % inch to meters

vhcl_infofile = '..//Data//Vehicle//Tesla_S_adapted';
testrun_infofile = '..//Data//TestRun//testrun_building';


%% Declaration of the structures of parameters

% vehicle parameters
vhcl = struct;

% sensors
% steering angle sensor
steer_sensor = struct;
% inertial measurement unit with gyros and accelerometers
IMU = struct;
% magnetometer
mag = struct;
% road property sensor
RP = struct;
% ctrl law definition
ctr_law = struct;
% low level controllers def
low_ctr = struct;



%% CarMaker Parameters reading
% Prepares parameters reading from CarMaker infofiles

% infofile handle for vehicle parametrization
ifid_vhcl = ifile_new();
ifile_read(ifid_vhcl, vhcl_infofile);

% infofile handle for testrun (environment, roads, ...)
ifid_testrun = ifile_new();
ifile_read(ifid_testrun, testrun_infofile);

% get steering mode
% steer_mode = ifile_getstr(ifid_vhcl, 'Steering.Kind');

% sampling frequency of CarMaker (Hz)
Fs_CM = 1000;

%% Vehicle parameters

% need to correspond to "vhcl" bus definition

% vehicle wheelbase (m)
% is given in mm in the infofile
vhcl.wheelbase = 1e-3 * ifile_getnum(ifid_vhcl,'CarGen.Vehicle.WheelBase');

% x-position of rear axle
% "getnum" takes only the first coordinate
vhcl.rear_axle_x = ifile_getnum(ifid_vhcl, 'WheelCarrier.rr.pos');
% x-pos of front axle
vhcl.front_axle_x = ifile_getnum(ifid_vhcl, 'WheelCarrier.fr.pos');

% wheel radius (m)
w_dims = str2num(ifile_getstr(ifid_vhcl,'CarGen.Vehicle.WheelSize')); %#ok<ST2NM>
% first coord: tyre width (mm)
% second: tyre aspect ratio in percent
% third: rim diameter (inches)
vhcl.wheel_radius = w_dims(3)/2 * in2m + 1e-3*w_dims(1)*(w_dims(2)/100);

% vehicle mass (kg)
vhcl.mass = ifile_getnum(ifid_vhcl,'CarGen.Vehicle.Weight');

% gear ratio between motor and wheels
vhcl.gear_ratio = ...
    ifile_getnum(ifid_vhcl,'PowerTrain.GearBoxM.iForwardGears');


%% CarMaker model parameters
% these parameters concern the car model and are sent to CarMaker. They can
% be defined in the software but it is done here for centralization
% purposes. However other things (such as the road) have to be defined in
% the GUI.

% desired steering model (refer to CarMaker doc for more information)
% The steering mode corresponds to the parameter "Steering.Kind" in the
% vehicle infofile (cf relevant infofile handle). Only the angle mode has
% been implememented at the moment.
% - steer_flag = 0 : angle steering
% - steer_flag = 1 : torque steering
steer_flag = 0;

% defines corresponding steer name for CM
if steer_flag == 0
    steer_mode = 'GenAngle 1';
elseif steer_flag == 1;
    steer_mode = 'GenTorque 1';
else
    % bug
    throw(MException('Steer:BadFlag','Invalid specified steer model'))
end



%% Sensor parameters

% The sensors that are on the VIPALAB are described in [Vilca2015] (thesis)
% in the appendix G (p.180)

% Odometers ------------------
% a) motor odometry - gives translational speed
% sample frequency (Hz)
odo.motor_sensor_Fs = 50;
% quantization in m/s
odo.motor_sensor_err = 0.1;
% max speed measurable (arbitrary) (in m/s)
odo.motor_max_speed = 50;
% b) wheel encoders


% Steering angle -----------------------
% analog sensor on the VIPALABs so the high Fs of CarMaker is kept
% sampling frequency
steer_sensor.Fs = 50; % inherited from CarMaker
% number of bits
steer_sensor.nbits = 12;
% range (in rad)
steer_sensor.range = d2r * [-30 30];

% IMU ----------------------------------
% model: Xsens Mti10
% sub structures definition
IMU.gyro = struct;
% sampling frequency of IMU
IMU.Fs = 2000;
% latency (s)
IMU.latency = 2e-3;
% gyro range (in rad/s) UNKNOWN, just put more than enough range
IMU.gyro.range = d2r * [-360 360];
% gyro resolution (rad/s)
IMU.gyro.res = d2r * 0.2;

% Magnetometer -------------------------
% model: Freescale MMA7361LCR1 or Melexis MLX90609-N2
% sampling frequency
mag.Fs = 50;


% RTK-GPS ------------------------------


% regular GPS --------------------------


% range sensor (LIDAR) -----------------
% can be emulated by a "Free Space sensor" in CarMaker


% Road Property sensor -------------------
% CAUTION: this is an emulated sensor by CarMaker. It gives perfect
% information and does not correspond to reality. On the vipalabs, the
% lateral errors information is given by a fusion of the infos of the real
% sensors (image, gps, imu, ...).
% The 
% get RP sensor name
RP.name = ifile_getstr(ifid_vhcl, 'RPSensor.0.name');
% checks name 
% if not 'RP00', simulink won't get the signals
if not(strcmp(RP.name, 'RP00'))
    throw(MException('RP:BadName','Wrong RPSensor name. Should be RP00'))
end
% RP sensor position in car frame (origin at rear)
RP.pos = str2num(ifile_getstr(ifid_vhcl, 'RPSensor.0.pos')); %#ok<ST2NM>
% sampling frequency of RP sensor
RP.Fs = 50;
% distance of preview point (m)
% as of now, the preview point distance is fixed for a simulation
RP.preview_dist = 0;


%% Simulink model switches

% de-activates CM driver (if at false, no autonomous driving possible)
% autonomous_driving = true; % to implement

% activates lateral control
lateral_control = true;

% activates longitudinal control
longit_control = true;


%% Trajectory tracking control law parameters

% data types must match the ones indicated in "bus_definitions.m" for the
% ctr_law_params_bus

% desired control law to be used
% 1 : Vilca controller
% 2 : Stanley controller
% 3 : Kinematic controller (based on kinematic system in chained form)
ctr_law.flag = 3;
% convert to int
ctr_law.flag = int8(ctr_law.flag);

% gains for Vilca controller
% K = [K_d K_l K_o K_x K_RT K_theta]
% ctr_law.K_vilca = [1, 2.2, 8, 0.1, 0.01, 0.6]; % def of p.74
ctr_law.K_vilca = [0.1, 0.6, 10, 0.1, 0.3, 0.01]; % def of p.71
% gains for Stanley controller
% K_stanley = [k_main, k_d_yaw k_soft k_d_steer]
ctr_law.K_stanley = [2.5 0.1 1 1];
% gain for chained form kinematic controller
ctr_law.K_kin = 0.2;

% target speed (m/s)
ctr_law.target_speed = 8;

% distance to target in body frame 
% (in case of dynamic target which distance is fixed wrt the car)
ctr_law.d_x_target = RP.preview_dist;

% if Stanley controller, the error must be computed at front axle
% prev point x-position to front axle
if ctr_law.flag == 2
    pos_to_f_axle = RP.pos(1) + ctr_law.d_x_target - vhcl.front_axle_x;
    % checks preview distance w.r.t. the control law
    assert(abs(pos_to_f_axle) < 0.5 )
end

% TODO: for Kinematic controller, the data should ideally be at rear axle

% min and max speed for control law (m/s)
ctr_law.v_min = 1;
ctr_law.v_max = 10;

% max absolute steer angle for wheels (rad)
ctr_law.gamma_max = 30 * d2r;

% Control law execution rate (Hz)
ctr_law.Fs = 50;


%% Low level control law parameters
% (steering angle and speed tracking)

% sampling rate (same as traj tracking controller)
low_ctr.Fs = ctr_law.Fs;
% proportional corrector for acceleration controller
low_ctr.K_accel = 0.1;
% max accel asked to the motor/brakes (m/s^2)
low_ctr.max_accel = 5;

% gas pedal to asked torque mapping
% checks gas pedal mapping mode. Only formula embedded in low level control
% at the moment is a linear one.
% remark : in linear mode it seems that the minimal torque at zero gas is
% always zero, whatever the "trq_zero" value below. It is a purely linear
% mapping without offset.
mode = ifile_getstr(ifid_vhcl, 'PowerTrain.Control.GasInterpret.Mode');
if ~strcmp(mode, 'Linear')
    throw(MException(...
        'RP:BadGasMap',...
        'Non supported gas pedal to torque mapping'))
end
% asked torque at full throttle
low_ctr.trq_full = ifile_getnum(ifid_vhcl, ...
    'PowerTrain.Control.GasInterpret.TrqFull');
% asked torque at zero throttle
low_ctr.trq_zero = ifile_getnum(ifid_vhcl, ...
    'PowerTrain.Control.GasInterpret.TrqZero');

% brake pedal to braking torque mapping (approx)
% the brake mode should be "Pedal Actuation" in CarMaker.
% pedal actuation to MC pressure
brake_to_pMC = ifile_getnum(ifid_vhcl,'Brake.System.PedalAct2pMC');
% MC pressure to torque
pMC_to_Trq_mat = ifile_gettab(ifid_vhcl,'Brake.System.pWB2Trq',4);
% overall mapping
% first coeff for a front wheel and second for a rear wheel
% the coeff should be multiplied by 2 to have the torque on an axle
low_ctr.brake_to_Trq = brake_to_pMC * pMC_to_Trq_mat([1,3]);

% steering angle to angle at steering wheel conversion factor
% (empirical)
% good matching until ~1.5*pi angle at steering wheel (then there are some
% non linear effects).
low_ctr.st_angle_to_st_wheel = 12.8;

%% Parameters computed from the input values in previous sections
% <<< DO NOT enter numerical values here >>>

% odometry ----------
odo.K_dn = ceil(Fs_CM / odo.motor_sensor_Fs);
% num of points for the range
odo.n_pts = floor(2*odo.motor_max_speed / odo.motor_sensor_err);
% range
odo.bnd_pts = linspace(-odo.motor_max_speed, odo.motor_max_speed, ...
    odo.n_pts);

% steer sensor ------
steer_sensor.K_dn = ceil(Fs_CM / steer_sensor.Fs);
% sensor resolution (range divided by number of values - 12bits)
steer_sensor.res = d2r * 60 / 2^steer_sensor.nbits;
% range data for quantization
steer_sensor.bnd_pts = linspace(steer_sensor.range(1), ...
    steer_sensor.range(2), 2^steer_sensor.nbits);
% IMU ---------------
IMU.K_dn = ceil(Fs_CM / IMU.Fs);
% nb points for the range
IMU.gyro.npts = IMU.gyro.range(2) - IMU.gyro.range(1) / IMU.gyro.res;
% boundary points for quantization
IMU.gyro.bnd_pts = linspace(IMU.gyro.range(1), ...
IMU.gyro.range(2), IMU.gyro.npts);
% nb samples of delay on non downsampled signal
IMU.offset = IMU.latency / (IMU.Fs^-1);
% Magnetometer ------
mag.K_dn = ceil(Fs_CM / mag.Fs);
% RP sensor ---------
% downsampling factor for lateral error
RP.K_dn = ceil(Fs_CM / RP.Fs);


%% Parameters to be modified in the CarMaker infofiles

% these parameters are defined in this script and need to be send to
% CarMaker through the configuration files).

% sets desired preview distance for RP sensor
ifile_setstr(ifid_vhcl, 'RPSensor.0.PreviewDist', RP.preview_dist);
% sets desired steering mode
ifile_setstr(ifid_vhcl, 'Steering.Kind', steer_mode);



% writes the contents of the handle to the file
ifile_write(ifid_vhcl, vhcl_infofile);

% destroys the infofile handles
ifile_delete(ifid_vhcl);
ifile_delete(ifid_testrun);


%% Other code to be executed before the simulation

% saves bus definitions
run('bus_definitions.m');






