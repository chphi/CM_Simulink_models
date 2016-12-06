%% Parameters of the simulink extension to the CarMaker model
% contains the parameters needed to study the influence of using dynamical
% models on the performance of the low level vehicle control algorithms.
%
%
% author: Charles Philippe (charles.philippe@cranfield.ac.uk)


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
Fs_steer = 1;


% Motor odometry -----------------------


% IMU ----------------------------------
Fs_imu = 1;


% RTK-GPS ------------------------------


% regular GPS --------------------------


% range sensor (LIDAR) -----------------



% road property sensor -------------------
% CAUTION: this is an emulated sensor by CarMaker. It gives perfect
% information and does not correspond to reality. On the vipalabs, the
% lateral errors information is given by a fusion of the infos of the real
% sensors (image, gps, imu, ...).
Fs_lat_err = 50;




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

% downsampling of inputs
dnsp_factor_gyros = 1;
dnsp_factor_accels = 1;
dnsp_factor_steer = 1;
dnsp_factor_lateral_err = 1;






