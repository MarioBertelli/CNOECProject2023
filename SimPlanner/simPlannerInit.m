%% Simulation Settings
Ts_simulation = 1e-3;
entire_simulation_duration = 6;
optimization_horizon_duration = 2;

num_time_steps = entire_simulation_duration / Ts_simulation;
%% Vehicles Settings
% Number of vehicles
N_simulated_cars = 8;  

% Initial x coordinate of every vehicle
x0=[5, 10, 8, 12, 70, 80, 72, 80];

% Constant velocity of every vehicle
vx_simulated_cars=[17, 18, 20, 22, 0, 25, 28, 29];

% Car width and length
car_width = 1.5;
car_length = 3;

%% Speed reference Settings 
V_ref = 90/3.6;

%% Lanes Settings
% Possible lanes references
y_lanes = [2,5,8];

% Initial lane (y coordinate) of every vehicle
lane0=[2, 2, 8, 8, 5, 8, 2, 5];
%lane0 = randsample(y_lanes, 8, true);

% Time instant at which every vehicle switches lane (leave 0 if not)
tSwitch=[num_time_steps/6, num_time_steps/5, num_time_steps/6, num_time_steps/2, num_time_steps/4, num_time_steps/3, num_time_steps/2, num_time_steps/1];
%tSwitch=[10, 0, 0, 0, ]

% Lane in which every vehicle goes at time specified into tSwitch
%laneSwitch=[2, 5, 2, 8, 2, 8, 5, 2];
laneSwitch = lane0;

%% Distance Ellipse Constraint Settings
% Define covariance matrix for the proximity ellipse cost function entry
C_proximity     =   [3  0;
                     0  1;];
% Define covariance matrix for the distance ellipse constraint
C_dist          =   [1.2  0;
                     0  6;];
