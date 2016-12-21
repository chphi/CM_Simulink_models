
% bus definitions for Simulink models. The definition of a bus is necessary to pass a structure to a
% "matlab function" block in a simulink model.

%% Vehicle parameters bus

clear elems;
i = 1;

% wheelbase
elems(i) = Simulink.BusElement;
elems(i).Name = 'wheelbase';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% rear axle x-position in car frame
elems(i) = Simulink.BusElement;
elems(i).Name = 'rear_axle_x';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% front axle x-position in car frame
elems(i) = Simulink.BusElement;
elems(i).Name = 'front_axle_x';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

%  wheel radius
elems(i) = Simulink.BusElement;
elems(i).Name = 'wheel_radius';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% mass
elems(i) = Simulink.BusElement;
elems(i).Name = 'mass';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% gear ratio
elems(i) = Simulink.BusElement;
elems(i).Name = 'gear_ratio';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;


vhcl_bus = Simulink.Bus;
vhcl_bus.Elements = elems;

%% IMU output bus

clear elems;
i = 1;

% gyro output
elems(i) = Simulink.BusElement;
elems(i).Name = 'Omega_0';
elems(i).Dimensions = 3;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% accels output
elems(i) = Simulink.BusElement;
elems(i).Name = 'Alpha_0';
elems(i).Dimensions = 3;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

imu_bus = Simulink.Bus;
imu_bus.Elements = elems;


%% Magnetometer output bus

clear elems
i = 1;

% 2D heading
elems(i) = Simulink.BusElement;
elems(i).Name = 'hdg';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

mag_bus = Simulink.Bus;
mag_bus.Elements = elems;

%% Road property sensor bus

clear elems;
i = 1;

% lateral pos to road center
elems(i) = Simulink.BusElement;
elems(i).Name = 'lat_pos_ctr_road';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% lateral error path (centre of lane)
elems(i) = Simulink.BusElement;
elems(i).Name = 'lat_pos_ctr_lane';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% heading error w.r.t. path heading at preview point
elems(i) = Simulink.BusElement;
elems(i).Name = 'hdg_err';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% path heading at preview point
elems(i) = Simulink.BusElement;
elems(i).Name = 'path_hdg';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% path curvature at preview point
elems(i) = Simulink.BusElement;
elems(i).Name = 'curve_xy';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% position of preview point in car frame with centre of rear axle as origin
elems(i) = Simulink.BusElement;
elems(i).Name = 'preview_pt_pos_Fr1';
elems(i).Dimensions = 3;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% curvilinear position of vehicle on path 
% (projection from origin of Fr0 which is not far from the rear axle)
elems(i) = Simulink.BusElement;
elems(i).Name = 's';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;


RPSensor_bus = Simulink.Bus;
RPSensor_bus.Elements = elems;


%% Steer angle sensor bus

clear elems;
i = 1;

% steering angle wheel 1
elems(i) = Simulink.BusElement;
elems(i).Name = 'steer_angle_1';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% steering angle wheel 2
elems(i) = Simulink.BusElement;
elems(i).Name = 'steer_angle_2';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% mean angle
elems(i) = Simulink.BusElement;
elems(i).Name = 'mean_angle';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

SteerSensor_bus = Simulink.Bus;
SteerSensor_bus.Elements = elems;

%% Odometry signals bus

clear elems;
i = 1;

% motor odometry
elems(i) = Simulink.BusElement;
elems(i).Name = 'vel_from_motor';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

OdoSensors_bus = Simulink.Bus;
OdoSensors_bus.Elements = elems;


%% Sensors outputs bus
% "bus of buses"

clear elems;
i = 1;

% element for IMU output
elems(i) = Simulink.BusElement;
elems(i).Name = 'imu_out';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'imu_bus';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% element for mag output
elems(i) = Simulink.BusElement;
elems(i).Name = 'mag_out';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'mag_bus';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% element for RPSensor output
elems(i) = Simulink.BusElement;
elems(i).Name = 'rpsensor_out';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'RPSensor_bus';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% element for steer angles sensor output
elems(i) = Simulink.BusElement;
elems(i).Name = 'steersensor_out';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'SteerSensor_bus';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% element for odo sensors output bus
elems(i) = Simulink.BusElement;
elems(i).Name = 'odo_out';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'OdoSensors_bus';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

SensorsOut_bus = Simulink.Bus;
SensorsOut_bus.Elements = elems;

%% ctr_law bus

% used to pass the ctr_law structure defined in "Parameters.m" to the
% "control laws" block in the simulink model. 

clear elems;
i = 1;

% flag for choosing the control law
elems(i) = Simulink.BusElement;
elems(i).Name = 'flag';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'int8';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% Vilca controller gains
elems(i) = Simulink.BusElement;
elems(i).Name = 'K_vilca';
elems(i).Dimensions = 6;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% Stanley controller gains
elems(i) = Simulink.BusElement;
elems(i).Name = 'K_stanley';
elems(i).Dimensions = length(ctr_law.K_stanley);
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% Target speed
elems(i) = Simulink.BusElement;
elems(i).Name = 'target_speed';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% distance to target
elems(i) = Simulink.BusElement;
elems(i).Name = 'd_x_target';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% v_min
elems(i) = Simulink.BusElement;
elems(i).Name = 'v_min';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% v_max
elems(i) = Simulink.BusElement;
elems(i).Name = 'v_max';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% gamma_max
elems(i) = Simulink.BusElement;
elems(i).Name = 'gamma_max';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% sample rate
elems(i) = Simulink.BusElement;
elems(i).Name = 'Fs';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;


ctr_law_params_bus = Simulink.Bus;
ctr_law_params_bus.Elements = elems;


%% Low level control law parameters 
% (inner loop for steering angle and speed)

clear elems;
i = 1;

% flag for choosing the control law
elems(i) = Simulink.BusElement;
elems(i).Name = 'Fs';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% acceleration controller ----------
% proportional coeff.
elems(i) = Simulink.BusElement;
elems(i).Name = 'K_accel';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;
% max acceleration (for saturation)
elems(i) = Simulink.BusElement;
elems(i).Name = 'max_accel';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;
% torque asked at max throttle
elems(i) = Simulink.BusElement;
elems(i).Name = 'trq_full';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;
% torque asked at zero throttle
elems(i) = Simulink.BusElement;
elems(i).Name = 'trq_zero';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;


% brake controller ----------------
elems(i) = Simulink.BusElement;
elems(i).Name = 'brake_to_Trq';
elems(i).Dimensions = 2;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

% steering controller -------------
elems(i) = Simulink.BusElement;
elems(i).Name = 'st_angle_to_st_wheel';
elems(i).Dimensions = 1;
elems(i).DimensionsMode = 'Fixed';
elems(i).DataType = 'double';
elems(i).SampleTime = -1;
elems(i).Complexity = 'real';
i = i + 1;

low_ctr_params_bus = Simulink.Bus;
low_ctr_params_bus.Elements = elems;















