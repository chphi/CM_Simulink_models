%% Parameters of the simulink extension to the CarMaker model
% contains the parameters needed to study the influence of using dynamical
% models on the performance of the low level vehicle control algorithms.
%
%
% author: Charles Philippe (charles.philippe@cranfield.ac.uk)

%% Constants

% declaration
global d2r
global r2d

% value
d2r = pi / 180;
r2d = 1 / d2r;

%% Declaration of the structures of parameters

% sensors
% steering angle sensor
steer_sensor = struct;
% inertial measurement unit with gyros and accelerometers
IMU = struct;
% road property sensor
RP = struct;



%% CarMaker Parameters needed before Simulink execution

% sampling frequency of CarMaker (Hz)
Fs_CM = 1000; 



%% CarMaker model parameters
% these parameters concern the car model and are sent to CarMaker. They can
% be defined in the software but it is done here for centralization
% purposes. However other things (such as the road) have to be defined in
% the GUI.

% steering model (steering ratio or full steering rack representation)
% steer_model = 

% The steering mode is given by the parameter "Steer.SteerBy", accessible
% via a "Read CM Dict" block.
% Output: 
% 1 : angle steering
% 2 : torque steering (when using Dynamic Steer Ratio or Pfeffer Model)


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


% road property sensor -------------------
% CAUTION: this is an emulated sensor by CarMaker. It gives perfect
% information and does not correspond to reality. On the vipalabs, the
% lateral errors information is given by a fusion of the infos of the real
% sensors (image, gps, imu, ...).
RP.Fs = 50;




%% Simulink model switches



% de-activates CM driver (if at false, no autonomous driving possible)
autonomous_driving = true;

% activates lateral control
lateral_control = true; % (TO IMPLEMENT)

% activates longitudinal control
longit_control = true; % (TO IMPLEMENT)

% discrete steering angle switch
discrete_steer_angle = true; % (TO IMPLEMENT IN SIMULINK)



















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

% RP sensor
dnsp_factor_lateral_err = 1;






