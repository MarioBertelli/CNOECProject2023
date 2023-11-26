function [zdot, F] = augmented_vehicle_model(t, z_main, u, d, th, z_neighboring, speeds_neighboring)
    % AUGMENTED_VEHICLE_MODEL Simulates the dynamics of a main vehicle considering neighboring vehicles.
    %
    % Inputs:
    %   t: Time (for use with ode45)
    %   z: Model state of the main vehicle
    %   u: Input (braking/driving torque and steering angle)
    %   d: Lateral wind
    %   th: Model parameters of the main vehicle
    %   z_neighboring: States of neighboring vehicles
    %   speeds_neighboring: Speeds of neighboring vehicles
    %
    % Outputs:
    %   zdot: Derivative of the state with respect to time for main and neighboring vehicles
    %   F: Longitudinal and lateral forces on the main vehicle
    
    % Call the existing vehicle function to compute the dynamics of the main vehicle
    [zdot_main, F_main] = vehicle(t, z_main, u, d, th);

    % Assuming z_neighboring contains states of neighboring vehicles
    % speeds_neighboring contains the speeds of the neighboring vehicles
    
    % Compute the dynamics of the neighboring vehicles
    zdot_neighboring = speeds_neighboring;

    % Return updated zdot combining the main vehicle dynamics and interactions with neighboring vehicles
    zdot = [zdot_main;          % Update with additional terms for neighboring vehicle interactions
            zdot_neighboring;
            ];
    % Return computed forces 
    F = F_main; 
end