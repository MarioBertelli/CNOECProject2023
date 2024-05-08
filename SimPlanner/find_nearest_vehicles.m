function [z_neighboring, speeds_neighboring] = find_nearest_vehicles(t, x_ego, y_ego, x_car_simulated, y_car_simulated, vx, Ts_simulation, Ts_optimization)
    % FIND_NEAREST_VEHICLES Finds the positions and speeds of the three nearest vehicles to the ego vehicle.
    % Usage:
    %   [z_neighboring, speeds_neighboring] = find_nearest_vehicles(t, x_ego, y_ego, x_car_simulated, y_car_simulated, vx, Ts_simulation)
    %
    % Inputs:
    %   t: Current time
    %   x_ego, y_ego: Ego vehicle's x and y positions
    %   x_car_simulated, y_car_simulated: Simulated positions of all vehicles over time
    %   vx: Speeds of all vehicles
    %   Ts_simulation: Time step used to generate simulated data
    %
    % Outputs:
    %   z_neighboring: Positions of the three nearest vehicles (x, y, theta for each)
    %   speeds_neighboring: Speeds of the three nearest vehicles
    
    % Convert time 't' to the index in the simulated data using interpolation
    t_index = round(t / Ts_simulation)*Ts_optimization;

    % Calculate distances based on the corresponding time index
    distances = sqrt((x_ego - x_car_simulated(t_index, :)).^2 + (y_ego - y_car_simulated(t_index, :)).^2);

    % Sort vehicles by distance
    [~, sorted_indices] = sort(distances);

    % Select the three nearest vehicles
    nearest_indices = sorted_indices(1:3);

    % Three states for each vehicle x, y, theta
    z_neighboring = zeros(3*3, 1);
    % Fill the initial x
    z_neighboring(1:3:end) = x_car_simulated(t_index, nearest_indices); 
    % Fill the initial y
    z_neighboring(2:3:end) = y_car_simulated(t_index, nearest_indices);
    % Near vehicles speed
    speeds_neighboring = zeros(size(z_neighboring));
    speeds_neighboring(1:3:end) = vx(nearest_indices);
end
