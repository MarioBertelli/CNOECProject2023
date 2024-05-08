%**************SCRIPT DESCRIPTION*************%

% This MATLAB script implements a Model Predictive Control (MPC) algorithm
% for autonomous vehicle trajectory planning and control. The MPC
% controller optimizes vehicle inputs over a prediction horizon to minimize
% a cost function while satisfying constraints, thereby enabling the
% vehicle to navigate a predefined environment safely and efficiently.

% The script is divided into several sections:

% 1. Initialization:
%    - Simulation parameters are initialized, including the simulation
%      environment settings and plant model parameters.
%    - Parameters for the Finite Horizon Optimal Control Problem (FHOCP) are
%      set, such as the optimization time step, prediction horizon, and
%      terminal time.

% 2. Simulation Planner:
%    - Simulates the planned trajectories of multiple vehicles in the
%      environment using SimPlanner, a predefined function.
%    - Determines the initial state of the ego vehicle and identifies the
%      nearest neighboring vehicles.

% 3. Optimization Variables and Constraints:
%    - Initializes optimization variables and defines constraints on inputs
%      (torque and steering angle) and track borders.

% 4. Solver Initialization and MPC Formulation:
%    - Sets up solver options for the optimization problem using the
%      Broyden-Fletcher-Goldfarb-Shanno (BFGS) method.
%    - Defines the cost function and constraints for the optimization
%      problem and runs the solver to obtain optimal inputs.
%    - Sets up the MPC formulation by defining the number of states and
%      inputs, as well as the number of MPC iterations.
%    - Iteratively runs the MPC algorithm to generate trajectory plans
%      while considering vehicle dynamics and environment constraints.

% 5. Visualize Results:
%    - Visualizes the simulated environment and trajectory of the ego
%      vehicle along with neighboring vehicles.

% 6. Debug and Report Plots:
%    - Generates plots for debugging and reporting purposes, including
%      ego vehicle trajectory, cost function entries, and other relevant
%      parameters.

% The script incorporates functions for vehicle trajectory simulation,
% cost function evaluation, and environment visualization. It also provides
% options for customization and fine-tuning of MPC parameters and
% visualization settings.
clear all
close all
clc
addpath('SimPlanner')
addpath('optimizer')
%% Simulation parameters initialization
% Simulation Environment Settings initialization
run('simPlannerInit.m');
% Plant Model Settings Initialization
run('plantModelInit.m');

%% FHOCP parameters - single shooting
% Optimization time step
Ts_optimization      =       0.25;                            % seconds, input sampling period
% Terminal time
Tend                 =       optimization_horizon_duration;   % seconds, terminal time
% Prediction horizon
Np                   =       Tend/Ts_optimization;            % prediction horizon

%% Simulation Planner
% Obtain x and y coordinate matrices of the N vehicles based on SimPlanner settings
[x_simulated_cars, y_simulated_cars] = simPlanner(x0, lane0, vx_simulated_cars, tSwitch, laneSwitch, N_simulated_cars, entire_simulation_duration, Ts_simulation);

% Calculate distances between ego vehicle and all other vehicles
t=1;
x_ego = [z0_main(1)];
y_ego = [z0_main(2)];
[z0_neighboring, speeds0_neighboring] = find_nearest_vehicles(t, z0_main(1), z0_main(2), x_simulated_cars, y_simulated_cars, vx_simulated_cars, Ts_simulation, Ts_optimization);


%% Initialize optimization variables
% Initial torque value
torque_0_init = 0;
% Initial optimization variable vector [U_Cm, U_delta]
x0      =       [torque_0_init*ones(Np,1);      % Torque (Nm),
                zeros(Np,1)];                   % Steering angle (rad)

%% Initial model state varialbles
% State variables: [X Y Ux beta psi r | x_v0, y_v0, theta_v | x_v1, y_v1,
% theta_v1 | x_v2, y_v2, theta_v2]
% [State of the ego vehicle | state of nearest vehicle | state of the second
% nearest vehicle | state of the third nearest vehile ]
z0 = [z0_main;
      z0_neighboring];

%% Constraints
% Bounds on input variables
% Torque and delta bounds
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
% Number of states of the extended model (ego + other vehicles)
nz=length(z0);
% Number of inputs
nu=2; 
% Number of iteration of the MPC into the total simulation duration
N_mpc_sim = entire_simulation_duration / Ts_optimization;
% Initialize vectors to store states and inputs for all iterations
Zsim_MPC            =   zeros((N_mpc_sim+1)*nz,1);  % states
Usim_MPC            =   zeros(N_mpc_sim,nu);        % inputs

% Initial states
Zsim_MPC(1:nz,1)    =   z0;  
zt                  =   z0;     % Initial state for MPC iteration

% Initialize vectors to store cost function entries
delta_diff= zeros(N_mpc_sim,1); 
Td_diff= zeros(N_mpc_sim,1); 
heading_error= zeros(N_mpc_sim,1); 
lateral_error= zeros(N_mpc_sim,1); 
speed_error= zeros(N_mpc_sim,1); 
proximity= zeros(N_mpc_sim,1); 

% Iteration of the optimization process N_mpc_sim times
for ind=1:N_mpc_sim
    % Compute nearest vehicles states at current iteration, needed for constraints and costs definition
    [z_neighboring, speeds_neighboring] = find_nearest_vehicles(ind, zt(1), zt(2), x_simulated_cars, y_simulated_cars, vx_simulated_cars, Ts_simulation, Ts_optimization);
    
    % Add neighboring vehicle states to initial states of the ego vehicle
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

    % Extract control variables for the next step 
    u_actual = xstar(1:length(xstar)/nu:end,1);

    % Store control variables for current iteration
    Usim_MPC(ind,:) =   u_actual;
    
    % Simulate the model for one step using the obtained control inputs
    [z_sim] = Vehicle_traj(u_actual,Ts_optimization,1,th,zt(1:6), Ts_simulation);
    
    % Update initial state for next MPC iteration
    zt(1:6) = z_sim(:,end);

    % Update state at current MPC iteration
    Zsim_MPC((ind)*nz+1:(ind+1)*nz,1) = zt; 

    % Update initial guess for optimization variables of next iteration
    x0=[xstar(2:end/2); xstar(end/2); xstar(end/2+2:end); xstar(end)];

end

%% Visualize results
% Flatten Usim_MPC for visualization
flat_Usim_MPC = reshape(Usim_MPC, [], 1);

% Simulate trajectory
[z_sim] = Vehicle_traj(flat_Usim_MPC,Ts_optimization,N_mpc_sim,th,Zsim_MPC(1:6), Ts_simulation);

% Visualize environment
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

%% Debug and report plots
% Create figure for debug and report plots
debugFig=figure;
set(debugFig, 'Position', [0, 0, 1200, 2100]); % Adjust the size as needed
fname = 'your figure';
% Define a rescaling space vector for MPC costs on space
xMPC=Zsim_MPC(1:15:end,:);
timeScaleMPC=1:xMPC(end)/N_mpc_sim:xMPC(end);

picturewidth = 30; % set this parameter and keep it forever
hw_ratio = 0.65; % feel free to play with this ratio
set(findall(debugFig,'-property','FontSize'),'FontSize',22) % adjust fontsize to your document

set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
%set(debugFig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
set(gca,'TickLabelInterpreter','latex','FontSize',14);
print(debugFig,fname,'-dpng','-painters')

subplot(7,1,1);plot(X_sim,Y_sim,X_sim,Ymin,'k',X_sim,Ymax,'k',X_sim,YfirstLine,'k--',X_sim,YsecondLine,'k--','LineWidth',3),grid on
title('\textbf{Ego vehicle trajectory}');
xlabel('X [$m$]','Interpreter','latex');
ylabel('Y [$m$]','Interpreter','latex');
set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
print(debugFig,fname,'-dpng','-painters');

subplot(7,1,2);plot(timeScaleMPC',delta_diff,'LineWidth',3);title('\textbf{Delta diff}','Interpreter','latex');grid on;
xlabel('X [$m$]','Interpreter','latex');
ylabel('$delta Diff$ [rad/s]','Interpreter','latex');
set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
print(debugFig,fname,'-dpng','-painters');

subplot(7,1,3);plot(timeScaleMPC',Td_diff,'LineWidth',3);title('\textbf{Td diff}','Interpreter','latex');grid on;
xlabel('X [$m$]','Interpreter','latex');
ylabel('$Td Diff$ [$Nm/s$]','Interpreter','latex');
set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
print(debugFig,fname,'-dpng','-painters');

subplot(7,1,4);plot(timeScaleMPC',heading_error,'LineWidth',3);title('\textbf{Heading error}','Interpreter','latex');grid on;
xlabel('X [$m$]','Interpreter','latex');
ylabel('$Heading Error$ [rad]','Interpreter','latex');
set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
print(debugFig,fname,'-dpng','-painters');

subplot(7,1,5);plot(timeScaleMPC',lateral_error,'LineWidth',3);title('\textbf{Lateral error}','Interpreter','latex');grid on;
xlabel('X [$m$]','Interpreter','latex');
ylabel('$Lateral Error$ [$m$]','Interpreter','latex');
set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
print(debugFig,fname,'-dpng','-painters');

subplot(7,1,6);plot(timeScaleMPC',speed_error,'LineWidth',3);title('\textbf{Speed error}','Interpreter','latex');grid on;
xlabel('X [$m$]','Interpreter','latex');
ylabel('$Speed Error$ [$m/s$]','Interpreter','latex');
set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
print(debugFig,fname,'-dpng','-painters');

subplot(7,1,7);plot(timeScaleMPC',proximity,'LineWidth',3);title('\textbf{Proximity}','Interpreter','latex');grid on;
xlabel('X [$m$]','Interpreter','latex');
ylabel('Proximity [$m$]','Interpreter','latex');
set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
print(debugFig,fname,'-dpng','-painters');

% secondFig=figure;
% set(secondFig, 'Position', [0, 0, 1200, 2100/7*3]); % Adjust the size as needed
% 
% subplot(3,1,1);plot(X_sim,Y_sim,X_sim,Ymin,'k',X_sim,Ymax,'k',X_sim,YfirstLine,'k--',X_sim,YsecondLine,'k--','LineWidth',3),grid on
% title('\textbf{Ego vehicle trajectory}','Interpreter','latex');
% xlabel('X [$m$]','Interpreter','latex');
% ylabel('Y [$m$]','Interpreter','latex');
% set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
% set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
% set(gca,'TickLabelInterpreter','latex','FontSize',16);
% print(debugFig,fname,'-dpng','-painters');
% 
% subplot(3,1,2);plot(timeScaleMPC',speed_error,'LineWidth',3);title('\textbf{Speed error}','Interpreter','latex');grid on;
% xlabel('X [$m$]','Interpreter','latex');
% ylabel('$Speed Error$ [$m/s$]','Interpreter','latex');
% set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
% set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
% set(gca,'TickLabelInterpreter','latex','FontSize',16);
% print(debugFig,fname,'-dpng','-painters');
% 
% subplot(3,1,3);plot(timeScaleMPC',proximity,'LineWidth',3);title('\textbf{Proximity}','Interpreter','latex');grid on;
% xlabel('X [$m$]','Interpreter','latex');
% ylabel('Proximity [$m$]','Interpreter','latex');
% set(findall(debugFig,'-property','Interpreter'),'Interpreter','latex') 
% set(findall(debugFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
% set(gca,'TickLabelInterpreter','latex','FontSize',16);
% print(debugFig,fname,'-dpng','-painters');
% trajFig=figure;
% plot(X_sim,Y_sim,X_sim,Ymin,'k',X_sim,Ymax,'k',X_sim,YfirstLine,'k--',X_sim,YsecondLine,'k--','LineWidth',3),grid on
% title('Ego vehicle trajectory');
% xlabel('X [$m$]','Interpreter','latex');
% ylabel('Y [$m$]','Interpreter','latex');
% set(findall(trajFig,'-property','Interpreter'),'Interpreter','latex') 
% set(findall(trajFig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
% set(gca,'TickLabelInterpreter','latex','FontSize',16);
% print(trajFig,fname,'-dpng','-painters');
