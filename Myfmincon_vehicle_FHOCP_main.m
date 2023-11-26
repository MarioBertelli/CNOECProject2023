% Constrained Numerical Optimization for Estimation and Control
% Laboratory session G
% Script to test the constrained optimization algorithm on a Finite Horizon
% Optimal Control Problem for vehicle trajectory optimization

clear all
close all
clc

% Plant model parameters initialization
run('simPlannerInit.m');
run('plantModelInit.m');

%Call function that gives x and y coordinate matrices of the N vehicles
%based on previous settings
[x, y] = simPlanner(x0, lane0, vx, tSwitch, laneSwitch, N, num_time_steps, Ts_simulation);

 % Calculate distances between ego vehicle and all other vehicles
t=1;
x_ego = [z0_main(1)];
y_ego = [z0_main(2)];
distances = sqrt((x_ego(t) - x(t, :)).^2 + (y_ego(t) - y(t, :)).^2);

% Sort vehicles by distance
[~, sorted_indices] = sort(distances);

% Select the three nearest vehicles
nearest_indices = sorted_indices(1:3);
z_neighboring = zeros(3*3,1); %3 states for each vehicle x,y,theta
z_neighboring(1:3:end) = x(t, nearest_indices); % fill the initial x
speeds_neighboring = zeros(size(z_neighboring));
speeds_neighboring(1:3:end) = vx(nearest_indices);

%% FHOCP parameters - single shooting
Ts      =       0.5;                % seconds, input sampling period
Tend    =       simulation_duration;% seconds, terminal time
Np      =       Tend/Ts;            % prediction horizon

%% Initialize optimization variables
% Optimization variables: [U_Cm, U_delta]
torque_0_init = 0;
x0      =       [torque_0_init*ones(Np,1);     % Torque (Nm),
                zeros(Np,1)];       % Steering angle (rad)

%% Initial model state varialbles
z0 = [z0_main;
      z_neighboring];
%% Speed reference
V_ref = 80/3.6;
%% Constraints
% Bounds on input variables
C       =       [-eye(2*Np)
                eye(2*Np)];
d       =       [-Tdmax*ones(Np,1);
                 -dmax*ones(Np,1);
                 Tdmin*ones(Np,1);
                 dmin*ones(Np,1)];

% Number of nonlinear inequality constraints (track borders)
q       =       5*Np;

%% Solution -  BFGS
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e2;
myoptions.xsequence     =	'on';
myoptions.outputfcn     =   @(x)Vehicle_traj(x,Ts,Np,th, z0_main, Ts_simulation);

% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfmincon(@(x)Vehicle_cost_constr(x,Ts,Np,th,z0,speeds_neighboring, y_lanes,V_ref),x0,[],[],C,d,0,q,myoptions);



%% Visualize results
[z_sim] = Vehicle_traj(xstar,Ts,Np,th,z0, Ts_simulation);

envVisualization(x,y, z_sim(1,:)',z_sim(2,:)',z_sim(5,:)');
