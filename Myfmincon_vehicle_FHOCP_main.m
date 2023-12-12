%**************SCRIPT DESCRIPTION*************%
%**************SCRIPT DESCRIPTION*************%
%**************SCRIPT DESCRIPTION*************%
%**************SCRIPT DESCRIPTION*************%

clear all
close all
clc

%% Simulation parameters initialization
% Simulation Environment Settings init
run('simPlannerInit.m');
% Plant Model Settings Init
run('plantModelInit.m');

%% Simulation Planner
% Call function that gives x and y coordinate matrices of the N vehicles based on previous settings
[x_simulated_cars, y_simulated_cars] = simPlanner(x0, lane0, vx_simulated_cars, tSwitch, laneSwitch, N_simulated_cars, entire_simulation_duration, Ts_simulation);

% Calculate distances between ego vehicle and all other vehicles
t=1;
x_ego = [z0_main(1)];
y_ego = [z0_main(2)];
[z_neighboring, speeds_neighboring] = find_nearest_vehicles(t, z0_main(1), z0_main(2), x_simulated_cars, y_simulated_cars, vx_simulated_cars, Ts_simulation);

%% FHOCP parameters - single shooting
Ts_optimization      =       0.5;                % seconds, input sampling period
Tend    =       optimization_horizon_duration;% seconds, terminal time
Np      =       Tend/Ts_optimization;            % prediction horizon

%% Initialize optimization variables
% Optimization variables: [U_Cm, U_delta]
torque_0_init = 0;
x0      =       [torque_0_init*ones(Np,1);     % Torque (Nm),
                zeros(Np,1)];       % Steering angle (rad)

%% Initial model state varialbles
z0 = [z0_main;
      z_neighboring];

%% Constraints
% Bounds on input variables
C       =       [
                -eye(2*Np)
                eye(2*Np)
                ];
d       =       [
                -Tdmax*ones(Np,1);
                 -dmax*ones(Np,1);
                 Tdmin*ones(Np,1);
                 dmin*ones(Np,1)
                ];

% Number of nonlinear inequality constraints (track borders)
q       =       6*Np + 2*(Np-1);    % + 2*(Np-1) are the delta_diff constraints

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
myoptions.outputfcn     =   @(x)Vehicle_traj(x,Ts_optimization,Np,th, z0_main, Ts_simulation);

%% MPC Formulation
nz=length(z0);
nu=2; 
N_mpc_sim = entire_simulation_duration / Ts_optimization;
Zsim_MPC            =   zeros((N_mpc_sim+1)*nz,1); 
Usim_MPC            =   zeros(N_mpc_sim*nu,1); 
Zsim_MPC(1:nz,1)    =   z0;  
zt                  =   z0; 


for ind=1:N_mpc_sim
    % compute nearest vehicles at current iteration
    [z_neighboring, speeds_neighboring] = find_nearest_vehicles(t, zt(1), zt(2), x_simulated_cars, y_simulated_cars, vx_simulated_cars, Ts_simulation);
    zt(7:end) = z_neighboring;
    % Run solver
    fun=@(x)Vehicle_cost_constr(x,Ts_optimization,Np,th,z0,speeds_neighboring,y_lanes,V_ref,C_proximity,C_dist,car_width);
    [xstar,fxstar,niter,exitflag,xsequence] = myfmincon(fun,x0,[],[],C,d,0,q,myoptions);
    u_actual = xstar(1:length(xstar)/nu:end,1);
    Usim_MPC((ind-1)*nu+1:(ind)*nu,1) =   u_actual;
    u_actual = xstar(1:length(xstar)/2:end,1); 

    % Simulate-Integrate the model to get the next initial state, simulate
    % it considering 1 control input applied for a time equal to the period
    % of one mpc step
    [z_sim] = Vehicle_traj(xstar,Ts_optimization,1,th,zt, Ts_simulation);
    zt = z_sim(:,end);

    Zsim_MPC((ind)*nz+1:(ind+1)*nz,1) = zt; 
    x0=[xstar(2:end/2); xstar(end/2); xstar(end/2+2:end); xstar(end)];


end
% Cost and Constraints Definition Function
fun=@(x)Vehicle_cost_constr(x,Ts_optimization,Np,th,z0,speeds_neighboring,y_lanes,V_ref,C_proximity,C_dist,car_width);

% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfmincon(fun,x0,[],[],C,d,0,q,myoptions);

%% Visualize results
% Final Vehicle Trajectory Visualization
[z_sim] = Vehicle_traj(xstar,Ts_optimization,Np,th,z0, Ts_simulation);

% Custom App Call for environment visualization, animation and video saving
envVisualization(x_simulated_cars,y_simulated_cars, z_sim(1,:)',z_sim(2,:)',z_sim(5,:)', C_proximity, C_dist, car_width, car_length, y_lanes);

%% Debug plots
debugFig=figure;
set(debugFig, 'Position', [0, 270, 1500, 530]); % Adjust the size as needed
subplot(2,3,1);plot(delta_diff);title('delta diff');
subplot(2,3,2);plot(Td_diff);title('Td diff');
subplot(2,3,3);plot(heading_error);title('heading error');
subplot(2,3,4);plot(lateral_error);title('lateral error');
subplot(2,3,5);plot(speed_error);title('speed error');
subplot(2,3,6);plot(distance_nearest_vehicle_1);title('distance_nearest_vehicle_1');
