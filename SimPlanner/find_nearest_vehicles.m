function [z_neighboring, speeds_neighboring] = find_nearest_vehicles(t, x_ego, y_ego, x_simulated_cars, y_simulated_cars, vx_simulated_cars)
    distances = sqrt((x_ego - x_simulated_cars(t, :)).^2 + (y_ego - y_simulated_cars(t, :)).^2);

    % Sort vehicles by distance
    [~, sorted_indices] = sort(distances);

    % Select the three nearest vehicles
    nearest_indices = sorted_indices(1:3);

    % Three states for each vehicle x, y, theta
    z_neighboring = zeros(3*3, 1);
    % Fill the initial x
    z_neighboring(1:3:end) = x_simulated_cars(t, nearest_indices); 
    % Fill the initial y
    z_neighboring(2:3:end) = y_simulated_cars(t, nearest_indices);
    % Near vehicles speed
    speeds_neighboring = zeros(size(z_neighboring));
    speeds_neighboring(1:3:end) = vx_simulated_cars(nearest_indices);
end

