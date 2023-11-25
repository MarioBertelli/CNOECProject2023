function v = Vehicle_cost_constr(x,Ts,Np,th,z0,y_ref)
% Function that computes the trajectory of the vehicle model
% introduced in Lab. session A and returns the cost function
% and the non linear equality and inequality constraints

%% Build vector of inputs
t_in        =   [0:Ts:(Np-1)*Ts]';
u_in        =   [x(1:Np,1)';
                x(Np+1:end,1)'];

assignin('base','z0',z0);
assignin('base','t_in',t_in);
assignin('base','u_in',u_in);

%% Run simulation with FFD
time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

z_sim      =   zeros(6,Nsim_FFD);
z_sim(:,1) =   z0;
for ind=2:Nsim_FFD
    u                   =   u_in(:,1+floor(time_FFD(ind)/Ts));
    zdot               =   vehicle(0,z_sim(:,ind-1),u,0,th);
    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot;
end

X_sim       =   z_sim(1,1:Nblock:end)';
Y_sim       =   z_sim(2,1:Nblock:end)';
V_sim       =   z_sim(3,1:Nblock:end)';
PSI_sim     =   z_sim(5,1:Nblock:end)';
%% Compute path constraints h(x)
h           =   [-V_sim + 80/3.6];

%% Compute cost function f(x)
% y_ref [y0_ref, y1_ref, y2_ref]
delta_diff  =   (x(Np+2:end,1)-x(Np+1:end-1,1));
Td_diff     =   (x(2:Np,1)-x(1:Np-1,1));

final_X = X_sim(end,1);
heading_error = PSI_sim;
lateral_error = y_ref(3) - Y_sim;

f           =   -final_X + ...
                1e2*(delta_diff'*delta_diff)+ ...
                1e-5*(Td_diff'*Td_diff) + ...
                1e1*(heading_error'*heading_error) + ...
                1e-3*(lateral_error'*lateral_error);

%% Stack cost and constraints
v           =   [f;h];


