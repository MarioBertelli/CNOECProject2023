%% Simulation Settings
% Define simulation parameters including the time step, entire simulation
% duration, and optimization horizon duration.
Ts_simulation = 1e-3;
entire_simulation_duration = 8;
optimization_horizon_duration = 2;

% Calculate the number of time steps based on the simulation duration and
% time step.
num_time_steps = entire_simulation_duration / Ts_simulation;
%% Speed Reference Settings 
% Set the reference speed (V_ref) for the ego vehicle in meters per second.
V_ref = 120/3.6;

%% Vehicles Settings
% Set parameters for simulated vehicles:
% - Number of simulated vehicles (N_simulated_cars)
% - Initial x coordinates of each vehicle (x0)
% - Constant velocity of each vehicle (vx_simulated_cars)
% - Width and length of each vehicle (car_width, car_length)

N_simulated_cars = 8;  
x0=[8, 9, 8, 12, 70, 80, 72, 80]+40;
vx_simulated_cars=[7, 6, 5, 22, 10, 25, 28, 29];
car_width = 1.5;
car_length = 3;


%% Lanes Settings
% Define parameters related to lanes:
% - Possible lane references (y_lanes)
% - Initial lane (y coordinate) of each vehicle (lane0)
% - Time instants at which each vehicle switches lanes (tSwitch)
% - Lane to which each vehicle switches at specified time instants (laneSwitch)

y_lanes = [2,5,8];
lane0=[2, 5, 8, 8, 5, 8, 2, 5];
%lane0 = randsample(y_lanes, 8, true);
tSwitch=[num_time_steps/6, num_time_steps/5, num_time_steps/6, num_time_steps/2, num_time_steps/4, num_time_steps/3, num_time_steps/2, num_time_steps/1];
%tSwitch=[10, 0, 0, 0, ]
%laneSwitch=[2, 5, 2, 8, 2, 8, 5, 2];
laneSwitch = lane0;

%% Distance Ellipse Constraint Settings
% Define covariance matrices for:
% - Proximity ellipse cost function entry (C_proximity)
% - Distance ellipse constraint (C_dist)
C_proximity     =   [3  0;
                     0  1;];
C_dist          =   [1.2  0;
                     0  6;];
