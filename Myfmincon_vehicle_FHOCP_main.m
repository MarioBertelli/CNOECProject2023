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

%% FHOCP parameters - single shooting
Ts_optimization      =       0.5;                             % seconds, input sampling period
Tend                 =       optimization_horizon_duration;   % seconds, terminal time
Np                   =       Tend/Ts_optimization;            % prediction horizon

%% Simulation Planner
% Call function that gives x and y coordinate matrices of the N vehicles based on previous settings
[x_simulated_cars, y_simulated_cars] = simPlanner(x0, lane0, vx_simulated_cars, tSwitch, laneSwitch, N_simulated_cars, entire_simulation_duration, Ts_simulation);

% Calculate distances between ego vehicle and all other vehicles
t=1;
x_ego = [z0_main(1)];
y_ego = [z0_main(2)];
[z_neighboring, speeds_neighboring] = find_nearest_vehicles(t, z0_main(1), z0_main(2), x_simulated_cars, y_simulated_cars, vx_simulated_cars, Ts_simulation, Ts_optimization);


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
% Number of states of the extended model (ego+other vehicles)
nz=length(z0);
% Number of inputs
nu=2; 
% Number of iteration of the MPC into the total simulation duration
N_mpc_sim = entire_simulation_duration / Ts_optimization;
% Init of a vector big enough to contain all the states for all the iterations
Zsim_MPC            =   zeros((N_mpc_sim+1)*nz,1); 
% Init of a vector big enough to contain all the inputs for all the iterations
Usim_MPC            =   zeros(N_mpc_sim,nu); 
% States of the initial iteration are the initial states
Zsim_MPC(1:nz,1)    =   z0;  
% Zt filled with initial state
zt                  =   z0; 
% Cost Function entries vectors 
delta_diff= zeros(N_mpc_sim,1); 
Td_diff= zeros(N_mpc_sim,1); 
heading_error= zeros(N_mpc_sim,1); 
lateral_error= zeros(N_mpc_sim,1); 
speed_error= zeros(N_mpc_sim,1); 
proximity= zeros(N_mpc_sim,1); 

% Iteration of the optimization process N_mpc_sim times
for ind=1:N_mpc_sim
    % compute nearest vehicles states at current iteration, needed for constraints and costs definition
    [z_neighboring, speeds_neighboring] = find_nearest_vehicles(ind, zt(1), zt(2), x_simulated_cars, y_simulated_cars, vx_simulated_cars, Ts_simulation, Ts_optimization);
    % Adding to initial states of the ego vehicle the states of neighboring vehicles
    zt(7:end) = z_neighboring;
    % Define costs and constraints
    fun=@(x)Vehicle_cost_constr(x,Ts_optimization,Np,th,zt,speeds_neighboring,y_lanes,V_ref,C_proximity,C_dist,car_width);
    % Run solver
    try
        [xstar,fxstar,niter,exitflag,xsequence] = myfmincon(fun,x0,[],[],C,d,0,q,myoptions);
    catch
        break;
    end

    % Accumulate cost function entries values for every iteration
    delta_diff(ind)=delta_diff_temp;
    Td_diff(ind)= Td_diff_temp; 
    heading_error(ind)= heading_error_temp; 
    lateral_error(ind)= lateral_error_temp; 
    speed_error(ind)= speed_error_temp; 
    proximity(ind)= proximity_temp; 

    % Take from optimization variable vector only the value at first step of the horizon for each variable 
    u_actual = xstar(1:length(xstar)/nu:end,1);
    % For every iteration put 1-step horizon control variables into increasing position of Usim_MPC
    Usim_MPC(ind,:) =   u_actual;
    % Simulate-Integrate the model to get the next initial state, simulate
    % it considering 1 control input applied for a time equal to the period
    % of one mpc step
    [z_sim] = Vehicle_traj(u_actual,Ts_optimization,1,th,zt(1:6), Ts_simulation);
    % Assign to zt last ego vehicle state to be transmitted to next cicle
    zt(1:6) = z_sim(:,end);
    % Update state at current MPC iteration
    Zsim_MPC((ind)*nz+1:(ind+1)*nz,1) = zt; 
    % Initial guess for optimization variables of next iteration
    x0=[xstar(2:end/2); xstar(end/2); xstar(end/2+2:end); xstar(end)];

end

%% Visualize results
% Custom App Call for environment visualization, animation and video saving
flat_Usim_MPC = reshape(Usim_MPC, [], 1);
[z_sim] = Vehicle_traj(flat_Usim_MPC,Ts_optimization,N_mpc_sim,th,Zsim_MPC(1:6), Ts_simulation);
envVisualization(x_simulated_cars,y_simulated_cars, z_sim(1,:)',z_sim(2,:)',z_sim(5,:)', C_proximity, C_dist, car_width, car_length, y_lanes);

%% MPC States
%Plot ego vehicle trajectory
MPCFig=figure;
hold on;
plot(Zsim_MPC(1:15:end,:), Zsim_MPC(2:15:end,:),"Marker","+");
% plot(Zsim_MPC(7:15:end,:), Zsim_MPC(8:15:end,:),"Marker","*");
% Zsim_MPC(1:15:end,:)
% Zsim_MPC(2:15:end,:)
% Zsim_MPC(3:15:end,:)
% Zsim_MPC(4:15:end,:)
% Zsim_MPC(5:15:end,:)
% Zsim_MPC(6:15:end,:)
% Zsim_MPC(7:15:end,:)
% Zsim_MPC(8:15:end,:)
% Zsim_MPC(9:15:end,:)
% Zsim_MPC(10:15:end,:)
% Zsim_MPC(11:15:end,:)
% Zsim_MPC(12:15:end,:)
% Zsim_MPC(13:15:end,:)
% Zsim_MPC(14:15:end,:)
% Zsim_MPC(15:15:end,:)

%% Debug plots
debugFig=figure;
%Define a rescaling space vector for MPC costs on space
xMPC=Zsim_MPC(1:15:end,:)
timeScaleMPC=1:xMPC(end)/N_mpc_sim:xMPC(end);
set(debugFig, 'Position', [0, 270, 1500, 530]); % Adjust the size as needed
subplot(2,3,1);plot(timeScaleMPC',delta_diff);title('Delta diff');grid on;
subplot(2,3,2);plot(timeScaleMPC',Td_diff);title('Td diff');grid on;
subplot(2,3,3);plot(timeScaleMPC',heading_error);title('Heading error');grid on;
subplot(2,3,4);plot(timeScaleMPC',lateral_error);title('Lateral error');grid on;
subplot(2,3,5);plot(timeScaleMPC',speed_error);title('Speed error');grid on;
subplot(2,3,6);plot(timeScaleMPC',proximity);title('Proximity');grid on;
