close all
clear all
clc

% Generate random sample data with constant y-coordinates, different starting x-positions, and varying x-coordinates based on velocities
N = 8;  % Number of vehicles
num_time_steps = 50;  % Number of time steps

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

%Call function that gives x and y coordinate matrices of the N vehicles
%based on previous settings
[x, y] = simPlanner(x0, lane0, vx, tSwitch, laneSwitch, N, num_time_steps);

% Generate random constant y-coordinate for the ego vehicle
y_ego = repmat(randi(3), num_time_steps, 1);

% Generate random positions and velocities for the ego vehicle (single entry)
x_ego = cumsum(rand(num_time_steps, 1) * 0.3) + randi(5);  % Vary the increase based on time steps

% Generate random orientations for the ego vehicle (varying for each time step)
ego_orientation = (rand(num_time_steps, 1) - 0.5) * pi / 3.0;

% Call the envVisualization function with the sample data
envVisualization(x, y, x_ego, y_ego, ego_orientation);
