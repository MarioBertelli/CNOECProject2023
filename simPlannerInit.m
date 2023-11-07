% Generate random sample data with constant y-coordinates, different starting x-positions, and varying x-coordinates based on velocities
N = 8;  % Number of vehicles
num_time_steps = 100;  % Number of time steps

% Initial x coordinate of every vehicle
x0=[10, 20, 30, 40, 50, 60, 70, 80];
% Initial lane (y coordinate) of every vehicle
lane0=[1, 2, 3, 1, 2, 3, 1, 2];
% Constant velocity of every vehicle
vx=[10, 40, 30, 10, 20, 10, 15, 20];
% Time instant at which every vehicle switches lane (leave 0 if not)
tSwitch=[num_time_steps/8, num_time_steps/7, num_time_steps/6, num_time_steps/5, num_time_steps/4, num_time_steps/3, num_time_steps/2, num_time_steps/1];
%tSwitch=[10, 0, 0, 0, ]
% Lane in which every vehicle hoes at time specified into tSwitch
laneSwitch=[2, 1, 2, 2, 3, 2, 2, 3];