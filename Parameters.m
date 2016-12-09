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

% value
d2r = pi / 180;
r2d = 1 / d2r;

vhcl_infofile = '..//Data//Vehicle//Tesla_S_adapted';
testrun_infofile = '..//Data//TestRun//testrun_building';


%% Declaration of the structures of parameters

% sensors
% steering angle sensor
steer_sensor = struct;
% inertial measurement unit with gyros and accelerometers
IMU = struct;
% road property sensor
RP = struct;
% ctrl law definition
ctr_law = struct;



%% CarMaker Parameters reading
% parameters needing to be known before Simulink execution. Read from
% CarMaker infofiles.

% infofile handle for vehicle parametrization
ifid_vhcl = ifile_new();
ifile_read(ifid_vhcl, vhcl_infofile);

% infofile handle for testrun (environment, roads, ...)
ifid_testrun = ifile_new();
ifile_read(ifid_testrun, testrun_infofile);

% get steering mode
% steer_mode = ifile_getstr(ifid_vhcl, 'Steering.Kind');

% get RP sensor name
RP.name = ifile_getstr(ifid_vhcl, 'RPSensor.0.name');
% checks name 
% if not 'RP00', simulink won't get the signals
if not(strcmp(RP.name, 'RP00'))
    throw(MException('RP:BadName','Wrong RPSensor name. Should be RP00'))
end

% sampling frequency of CarMaker (Hz)
Fs_CM = 1000;



%% CarMaker model parameters
% these parameters concern the car model and are sent to CarMaker. They can
% be defined in the software but it is done here for centralization
% purposes. However other things (such as the road) have to be defined in
% the GUI.

% desired steering model (refer to CarMaker doc for more information)
% The steering mode corresponds to the parameter "Steering.Kind" in the
% vehicle infofile (cf relevant infofile handle). Only the angle mode has
% been implememented at the moment.
% - angle steering : "GenAngle 1"
% - torque steering : "GenTorque 1"
steer_mode = 'GenAngle 1';



%% Sensor parameters

% The sensors that are on the VIPALAB are described in [Vilca2015] (thesis)
% in the appendix G (p.180)

% Rear wheel odometer ------------------


% Steering angle -----------------------
% analog sensor on the VIPALABs so the high Fs of CarMaker is kept
% sampling frequency
steer_sensor.Fs = Fs_CM; % inherited from CarMaker
% number of bits
steer_sensor.nbits = 12;
% range (in rad)
steer_sensor.range = d2r * [-30 30];
% sensor resolution (range divided by number of values - 12bits)
steer_sensor.res = d2r * 60 / 2^steer_sensor.nbits;
steer_sensor.bnd_pts = linspace(steer_sensor.range(1), ...
    steer_sensor.range(2), 2^steer_sensor.nbits);

% Motor odometry -----------------------


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


% RTK-GPS ------------------------------


% regular GPS --------------------------


% range sensor (LIDAR) -----------------
% can be emulated by a "Free Space sensor" in CarMaker


% Road Property sensor -------------------
% CAUTION: this is an emulated sensor by CarMaker. It gives perfect
% information and does not correspond to reality. On the vipalabs, the
% lateral errors information is given by a fusion of the infos of the real
% sensors (image, gps, imu, ...).
% sampling frequency of RP sensor
RP.Fs = 50;
% distance of preview point (m)
% as of now, the preview point distance is fixed for a simulation
RP.preview_dist = 10;


%% Simulink model switches

% de-activates CM driver (if at false, no autonomous driving possible)
autonomous_driving = true;

% activates lateral control
lateral_control = true; % (TO IMPLEMENT)

% activates longitudinal control
longit_control = true; % (TO IMPLEMENT)

% discrete steering angle switch
discrete_steer_angle = true; % (TO IMPLEMENT IN SIMULINK)


%% Control law parameters

% data types must match the ones indicated in "bus_definitions.m" for the
% ctr_law_params_bus

% desired control law to be used
% 1 : Vilca controller
ctr_law.flag = int8(1);

% gains for Vilca controller
% K = [K_d K_l K_o K_x K_RT K_theta]
ctr_law.K_vilca = [1 2.2 8 0.1 0.01 0.6];

















%% Parameters computed from the input values in previous sections
% <<< DO NOT enter numerical values here >>>

% steer sensor
steer_sensor.K_dn = ceil(Fs_CM / steer_sensor.Fs);

% IMU
IMU.K_dn = ceil(Fs_CM / IMU.Fs);
% nb points for the range
IMU.gyro.npts = IMU.gyro.range(2) - IMU.gyro.range(1) / IMU.gyro.res;
% boundary points for quantization
IMU.gyro.bnd_pts = linspace(IMU.gyro.range(1), ...
IMU.gyro.range(2), IMU.gyro.npts);
% nb samples of delay on non downsampled signal
IMU.offset = IMU.latency / (IMU.Fs^-1);

% RP sensor
dnsp_factor_lateral_err = ceil(RP.Fs / IMU.Fs);


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






