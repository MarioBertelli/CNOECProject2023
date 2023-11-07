%% Model parameters
m       =       1715;               % vehicle mass (kg)
Jz      =       2700;               % vehicle moment of inertia (kg*m^2)
a       =       1.07;               % distance between center of gravity and front axle (m)
b       =       1.47;               % distance between center of gravity and rear axle (m)
Cf      =       95117;              % front axle cornering stiffness (N/rad)
Cr      =       97556;              % rear axle cornering stiffness (N/rad)
rw      =       0.303;              % wheel radius (m)
mu      =       1;                  % road friction coefficient
Tdmax   =       1715*1.7*0.303;     % maximum driving torque (N*m)
Tdmin   =       -1715*9.81*0.303;   % maximum braking torque (N*m)
dmax    =       35*pi/180;          % maximum steering angle (rad)
dmin    =       -35*pi/180;         % minimum steering angle (rad)
Af      =       1.9;                % vehicle front surface (m^2)
Al      =       3.2;                % vehicle lateral surface (m^2)
Cx      =       0.4;                % vehicle front aerodynamic drag coefficient
Rr      =       0.016*m*9.81/30;    % rolling resistance coefficient(N*s/m)
rho     =       1.2;                % air density (kg/m^3)
th      =       [m;Jz;a;b;Cf;Cr;rw;mu;Tdmax;Tdmin;dmax;dmin;Af;Al;Cx;Rr;rho];

%% Simulation: Initial state
X       =       0;          % inertial X position (m)
Y       =       0;          % inertial Y position (m)
Ux      =       1;          % body x velocity (m/s)
beta    =       0;          % sideslip angle (rad)
psi     =       0;          % yaw angle (rad)
r       =       0;          % yaw rate (rad/s)
z0     =       [X;Y;Ux;beta;psi;r];

%% Simulation: step amplitude
Td_step             =       Tdmax/2.5;
delta_step          =       dmax/15;
%% Simulation with ode45
% Time integration parameters
Ts_o45      =       1e-1;               % sampling time (s)
Tend_o45    =       100;                % final time (s)
tvec_o45    =       0:Ts_o45:Tend_o45;  % time vector (s)
% Initialize simulation output
N_o45               =       length(tvec_o45);   % number of samples
zout_o45            =       zeros(6,N_o45);     % matrix with states
uout_o45            =       zeros(2,N_o45);     % matrix with inputs
Fout_o45            =       zeros(6,N_o45);     % matrix with forces
zout_o45(:,1)       =       z0;
%uout_o45(:,1)       =       [Td_step;delta_step];
%Provvisory input maneuever TO BE ASSIGNED ACCORDING TO CONTROL STRATEGY
for i=1:(Tend_o45/Ts_o45+1)/3
    uout_o45(2,i)=delta_step;
end
for i=floor((Tend_o45/Ts_o45+1)/3):(Tend_o45/Ts_o45+1)/3*2
    uout_o45(2,i)=-delta_step;
end
for i=floor((Tend_o45/Ts_o45+1)/3*2):(Tend_o45/Ts_o45+1)-1
    uout_o45(2,i)=0;
end