%% Parameters of the simulink extension to the CarMaker model
% contains the parameters needed to study the influence of using dynamical
% models on the performance of the low level vehicle control algorithms.



%% Input variables from CarMaker
% Names of the variables to which CarMaker gives access to the simulink
% user. These are the inputs of the path following (and other) algorithms.
% These variables are described in detail in the reference manual of
% CarMaker (chapter 29: 'User Accessible Quantities')

% angular rates in body frame
% omega_in.x = 'BodySensor.BD00.Omega_0.x';
% omega_in.y = 'BodySensor.BD00.Omega_0.y';
% omega_in.z = 'BodySensor.BD00.Omega_0.z';

% useless, needs to put the name directly in the Simulink block.




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
% - 1 : angle steering
% - 2 : torque steering (when using Dynamic Steer Ratio or Pfeffer Model)

%% Simulink model parameters



% de-activates CM driver (if at false, no autonomous driving possible)
autonomous_driving = true;

% activates lateral control
lateral_control = true; % (TO IMPLEMENT)

% activates longitudinal control
longit_control = true; % (TO IMPLEMENT)

% discrete steering angle switch
discrete_steer_angle = true; % (TO IMPLEMENT IN SIMULINK)

