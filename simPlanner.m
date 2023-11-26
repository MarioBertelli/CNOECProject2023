function [x, y, vx] = simPlanner(x0, lane0, vx, tSwitch, laneSwitch, N, M, Ts)
% SIMPLANNER - Generate positions, speeds and yawRates of the N vehicles 
% present into the simulation to be visualized and used by the control block.
% Limitations: It allows only the definition of a starting lane for every
% vehicles and up to one only change at a certain time stamp. The reason is
% the limited complexity of the project that doesn't require more
% complexity than that.
    % Input:
    %   x0: Vector of the starting x coordinate of the other vehicles (Nx1)
    %   lane0: Vector of the starting lanes of the others vehicles (Nx1)
    %   vx: Vector of the constant speeds of the others vehicles (Nx1)
    %   tSwitch: Vector of the time instant M in which the N vehicles
    %       change the lane (lane specified into laneSwitch_n (Nx1)
    %   laneSwitch: Vector of the lane index in which the vehicles goes
    %       at time instant defined into tSwitch_n (Nx1)
    %   N: number of vehicles into the simulation (1x1)
    %   M: number of time steps (1x1)
    % Output:
    %   x: Matrix of x-coordinates of other vehicles at different time steps (MxN), where M is the number of time steps, and N is the number of vehicles.
    %   y: Matrix of y-coordinates of other vehicles at different time steps (MxN), corresponding to the x-coordinates.

%Initialization of x-coordinates of the N vehicles
for i=1:N
    x(1,i)=x0(i);
end
% x matrix creation: based on initial x coordinate and velocity the x
% trajectory is computed
for i=2:M % Indexing timesteps after the initial ones already defined
    for j=1:N % Indexing vehicles at time step i
        x(i,j)= x(i-1,j)+(Ts* vx(j));
    end
end

%Initialization of y-coordinates of the N vehicles
for i=1:N
    y(1,i)=lane0(i);
end
% y matrix creation: based on starting lane, time of switch and lane into
% the switch is done the y trajectroy is computed
for i=2:M % Indexing timesteps after the initial ones already defined
    for j=1:N % Indexing vehicles at time step i
        if(tSwitch(j)>i)
            y(i,j)=y(i-1,j);
        else
            y(i,j)=laneSwitch(j);
        end
    end
end

end

