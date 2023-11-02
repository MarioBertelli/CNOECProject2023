close all
clear all

% Generate random sample data with constant y-coordinates, different starting x-positions, and varying x-coordinates based on velocities
N = 8;  % Number of vehicles
num_time_steps = 50;  % Number of time steps

% Generate random y-coordinates (constant values for all time steps)
y = repmat(randi(N-2, 1, N)*2.0, num_time_steps, 1);

% Generate random velocities and radii for the vehicles (varying for each vehicle)
vx = bsxfun(@times, rand(num_time_steps, N), (1:N)) * 0.2; % Vary the increase based on the vehicle index

% Generate random starting x-positions for each 

start_x = rand(1, N) * 15;

% Calculate x-coordinates based on velocities, considering starting positions
x = cumsum(vx, 1) + repmat(start_x, num_time_steps, 1);

% Generate random constant y-coordinate for the ego vehicle
y_ego = repmat(randi(N-2)*2.0, num_time_steps, 1);

% Generate random positions and velocities for the ego vehicle (single entry)
x_ego = cumsum(rand(num_time_steps, 1) * 0.3) + randi(5);  % Vary the increase based on time steps

% Generate random orientations for the ego vehicle (varying for each time step)
ego_orientation = (rand(num_time_steps, 1) - 0.5) * pi / 3.0;

% Call the envVisualization function with the sample data
envVisualization(x, y, x_ego, y_ego, ego_orientation);
