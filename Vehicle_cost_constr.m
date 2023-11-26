function v = Vehicle_cost_constr(x,Ts,Np,th,z0, speeds_neighboring, y_ref,V_ref)
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

z_sim      =   zeros(6+9,Nsim_FFD);
z_sim(:,1) =   z0;

for ind=2:Nsim_FFD
    u                   =   u_in(:,1+floor(time_FFD(ind)/Ts));
    z_main = z_sim(1:6,ind-1);
    z_neighbouring = z_sim(7:end, ind-1);
    zdot               =   augmented_vehicle_model(0,z_main,u,0,th, ...
                            z_neighbouring, speeds_neighboring);
    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot;
end

X_sim       =   z_sim(1,:)';
Y_sim       =   z_sim(2,1:Nblock:end)';
V_sim       =   z_sim(3,1:Nblock:end)';
PSI_sim     =   z_sim(5,1:Nblock:end)';

distance_nearest_vehicle_1 = abs(z_sim(7,:)' - X_sim) + 0.1;    % TODO temp distance as x difference right now
distance_nearest_vehicle_2 = abs(z_sim(10,:)' -  X_sim) + 0.1;    % TODO temp distance as x difference right now
distance_nearest_vehicle_3 = abs(z_sim(13,:)' - X_sim) + 0.1;   % TODO temp distance as x difference right now
% interpolate distances to have a finer grane proximity definition
% distance_nearest_vehicle_1 = interp1(distance_nearest_vehicle_1, 1:)


delta_diff  =   (x(Np+1:end,1)-x(Np:end-1,1));

%% Compute path constraints h(x)
h           =   [-V_sim + 130/3.6;
                 Y_sim + 2;
                 -Y_sim + 10
                 delta_diff + 0.1
                 -delta_diff + 0.1];
 
%% Compute cost function f(x)
% y_ref [y0_ref, y1_ref, y2_ref]
delta_diff  =   (x(Np+1:end,1)-x(Np:end-1,1));
Td_diff     =   (x(2:Np-1,1)-x(1:Np-2,1));

heading_error = PSI_sim;
lateral_error = y_ref(3) - Y_sim;
speed_error = V_ref - V_sim;
proximity =  (1./distance_nearest_vehicle_1)'*(1./distance_nearest_vehicle_1) ...
            +(1./distance_nearest_vehicle_2)'*(1./distance_nearest_vehicle_2) ...
            +(1./distance_nearest_vehicle_3)'*(1./distance_nearest_vehicle_3);
% -2*e-2*final_X + ...
f           =   1e3*(delta_diff'*delta_diff)+ ...
                1e-5*(Td_diff'*Td_diff) + ...
                1e2*(heading_error'*heading_error) + ...
                5e-1*(lateral_error'*lateral_error) + ...
                5e-5*(speed_error'*speed_error) + ...
                1e1* proximity;


%% Stack cost and constraints
v           =   [f;h];


