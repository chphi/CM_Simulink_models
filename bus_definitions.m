
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

% gyro output
elems(1) = Simulink.BusElement;
elems(1).Name = 'Omega_0';
elems(1).Dimensions = 3;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';

% accels output
elems(2) = Simulink.BusElement;
elems(2).Name = 'Alpha_0';
elems(2).Dimensions = 3;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';

imu_bus = Simulink.Bus;
imu_bus.Elements = elems;


%% Magnetometer output bus

clear elems

% 2D heading
elems(1) = Simulink.BusElement;
elems(1).Name = 'hdg';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';

mag_bus = Simulink.Bus;
mag_bus.Elements = elems;

%% Road property sensor bus

clear elems;

% lateral pos to road center
elems(1) = Simulink.BusElement;
elems(1).Name = 'lat_pos_ctr_road';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';

% lateral error path (centre of lane)
elems(2) = Simulink.BusElement;
elems(2).Name = 'lat_pos_ctr_lane';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';

% heading error w.r.t. path heading at preview point
elems(3) = Simulink.BusElement;
elems(3).Name = 'hdg_err';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';

% path heading at preview point
elems(4) = Simulink.BusElement;
elems(4).Name = 'path_hdg';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';

% path curvature at preview point
elems(5) = Simulink.BusElement;
elems(5).Name = 'curve_xy';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';

% position of preview point in car frame with centre of rear axle as origin
elems(6) = Simulink.BusElement;
elems(6).Name = 'preview_pt_pos_Fr1';
elems(6).Dimensions = 3;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';


RPSensor_bus = Simulink.Bus;
RPSensor_bus.Elements = elems;


%% Steer angle sensor bus

clear elems;

% steering angle wheel 1
elems(1) = Simulink.BusElement;
elems(1).Name = 'steer_angle_1';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';

% steering angle wheel 2
elems(2) = Simulink.BusElement;
elems(2).Name = 'steer_angle_2';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';

% mean angle
elems(3) = Simulink.BusElement;
elems(3).Name = 'mean_angle';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';

SteerSensor_bus = Simulink.Bus;
SteerSensor_bus.Elements = elems;

%% Odometry signals bus

clear elems;

% motor odometry
elems(1) = Simulink.BusElement;
elems(1).Name = 'vel_from_motor';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';

OdoSensors_bus = Simulink.Bus;
OdoSensors_bus.Elements = elems;


%% Sensors outputs bus
% "bus of buses"

clear elems;

% element for IMU output
elems(1) = Simulink.BusElement;
elems(1).Name = 'imu_out';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'imu_bus';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';

% element for mag output
elems(2) = Simulink.BusElement;
elems(2).Name = 'mag_out';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'mag_bus';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';

% element for RPSensor output
elems(3) = Simulink.BusElement;
elems(3).Name = 'rpsensor_out';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'RPSensor_bus';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';

% element for steer angles sensor output
elems(4) = Simulink.BusElement;
elems(4).Name = 'steersensor_out';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'SteerSensor_bus';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';

% element for odo sensors output bus
elems(5) = Simulink.BusElement;
elems(5).Name = 'odo_out';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'OdoSensors_bus';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';

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















