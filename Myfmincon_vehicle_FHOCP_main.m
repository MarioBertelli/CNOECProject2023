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

%% FHOCP parameters - single shooting
Ts      =       0.5;                % seconds, input sampling period
Tend    =       10;                 % seconds, terminal time
Np      =       Tend/Ts;            % prediction horizon

%% Initialize optimization variables
% Optimization variables: [U_Cm, U_delta]
torque_0_init = 100;
x0      =       [torque_0_init*ones(Np,1);     % Torque (Nm),
                zeros(Np,1)];       % Steering angle (rad)

%% Initialize model states

%% Constraints
% Bounds on input variables
C       =       [-eye(2*Np)
                eye(2*Np)];
d       =       [-Tdmax*ones(Np,1);
                 -dmax*ones(Np,1);
                 Tdmin*ones(Np,1)/10;
                 dmin*ones(Np,1)];
             
% Number of nonlinear inequality constraints (track borders)
q       =       1*Np;

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
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'on';
myoptions.outputfcn     =   @(x)Vehicle_traj(x,Ts,Np,th, z0);

% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfmincon(@(x)Vehicle_cost_constr(x,Ts,Np,th,z0,y_lanes),x0,[],[],C,d,0,q,myoptions);



%% Visualize results
[z_sim] = Vehicle_traj(xstar,Ts,Np,th,z0);
