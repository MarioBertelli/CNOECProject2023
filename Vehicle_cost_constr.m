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
time_FFD    =   [0:0.01:(Np)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD) - 1;

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

XY_sim_complete    =   z_sim(1:2,:)';
XY_sim_near1_complete = z_sim(7:8,:)';
XY_sim_near2_complete = z_sim(10:11,:)';
XY_sim_near3_complete = z_sim(13:14,:)';

XY_sim       =   z_sim(1:2,1:Nblock:end)';
V_sim       =   z_sim(3,1:Nblock:end)';
PSI_sim     =   z_sim(5,1:Nblock:end)';

XY_near1_sim = z_sim(7:8,1:Nblock:end)';
XY_near2_sim = z_sim(10:11,1:Nblock:end)';
XY_near3_sim = z_sim(13:14,1:Nblock:end)';

% compute mahalanobis distance from near vehicles 
C = [3  0;
     0  1;]; % TODO TUNATA A CASO, car aspect ratio?
distance_nearest_vehicle_1 = sqrt(sum((XY_sim_near1_complete - XY_sim_complete)*inv(C).*(XY_sim_near1_complete - XY_sim_complete),2));
distance_nearest_vehicle_2 = sqrt(sum((XY_sim_near2_complete - XY_sim_complete)*inv(C).*(XY_sim_near2_complete - XY_sim_complete),2));
distance_nearest_vehicle_3 = sqrt(sum((XY_sim_near3_complete - XY_sim_complete)*inv(C).*(XY_sim_near3_complete - XY_sim_complete),2));
% avoid division by 0:
distance_nearest_vehicle_1 = distance_nearest_vehicle_1 + 0.05;
distance_nearest_vehicle_2 = distance_nearest_vehicle_2 + 0.05;
distance_nearest_vehicle_2 = distance_nearest_vehicle_2 + 0.05;

delta_diff  =   (x(Np+2:end,1)-x(Np+1:end-1,1));
Td_diff = (x(2:Np,1)-x(1:Np-1,1));

car_width = 1.5;
%% Compute path constraints h(x)
h           =   [-V_sim + 130/3.6;
                 XY_sim(:,2) + 2 - car_width*1.3;
                 -XY_sim(:,2) + 10 - car_width * 1.3;
                 delta_diff + 0.1;
                 -delta_diff + 0.1];

% Ellipse minimum distance constraints:

% Define covariance matrix for the ellipse
C = [1.5 0; 0 3]; % Adjust based on your scenario

% Define the threshold (minimum distance from ego vehicle to ellipses)
threshold = 1; % Adjust this value based on your specific requirements

% Calculate distances between the ego vehicle and the three nearest vehicles
distances_to_nearest_vehicles = [
    sqrt(sum(((XY_sim - XY_near1_sim) * inv(C)) .* (XY_sim - XY_near1_sim), 2));
    sqrt(sum(((XY_sim - XY_near2_sim) * inv(C)) .* (XY_sim - XY_near2_sim), 2));
    sqrt(sum(((XY_sim - XY_near3_sim) * inv(C)) .* (XY_sim - XY_near3_sim), 2))
];

% Add constraints based on distances to ellipses of nearest vehicles
h_ellipse_nearest_vehicles = distances_to_nearest_vehicles - threshold;
h = [h;
     h_ellipse_nearest_vehicles];
if min(h) < 0
    a = 1;
end

% fig = figure;
% % Plot ego vehicle position
% plot(XY_sim(:,1), XY_sim(:,2), 'bo', 'MarkerSize', 2, 'DisplayName', 'Ego Vehicle');
% hold on;
% % Plot positions of nearest vehicles
% plot(XY_near1_sim(:,1), XY_near1_sim(:,2), 'ro', 'MarkerSize', 1, 'DisplayName', 'Nearest Vehicle 1', 'Color', "r");
% plot(XY_near2_sim(:,1), XY_near2_sim(:,2), 'go', 'MarkerSize', 1, 'DisplayName', 'Nearest Vehicle 2', 'Color', "g");
% plot(XY_near3_sim(:,1), XY_near3_sim(:,2), 'mo', 'MarkerSize', 1, 'DisplayName', 'Nearest Vehicle 3', 'Color', "b");
% 
% % Plot ellipses around the nearest vehicles
% for i = 1:Np
%     [V, D] = eig(C);
%     a = sqrt(threshold / D(1,1));
%     b = sqrt(threshold / D(2,2));
%     t = linspace(0, 2 * pi);
%     ellipse_x = a * cos(t);
%     ellipse_y = b * sin(t);
%     ellipse_points = V * [ellipse_x; ellipse_y];
%     plot(ellipse_points(1,:) + XY_near1_sim(i,1), ellipse_points(2,:) + XY_near1_sim(i,2), 'LineWidth', 1.5, 'DisplayName', ['Ellipse ', num2str(i)], 'Color', "r");
%     plot(ellipse_points(1,:) + XY_near2_sim(i,1), ellipse_points(2,:) + XY_near2_sim(i,2), 'LineWidth', 1.5, 'DisplayName', ['Ellipse ', num2str(i)], 'Color', "g");
%     plot(ellipse_points(1,:) + XY_near3_sim(i,1), ellipse_points(2,:) + XY_near3_sim(i,2), 'LineWidth', 1.5, 'DisplayName', ['Ellipse ', num2str(i)], 'Color', "b");
% 
% end
% 
% % legend;
% xlabel('X Position');
% ylabel('Y Position');
% title('Visualization of Ellipses around Nearest Vehicles');
% grid on;
% axis equal;
% hold off;

%% Compute cost function f(x)
% y_ref [y0_ref, y1_ref, y2_ref]

heading_error = PSI_sim;
lateral_error = y_ref(2) - XY_sim(:,2);
speed_error = V_ref - V_sim;
proximity =  (1./distance_nearest_vehicle_1)'*(1./distance_nearest_vehicle_1) ...
            +(1./distance_nearest_vehicle_2)'*(1./distance_nearest_vehicle_2) ...
            +(1./distance_nearest_vehicle_3)'*(1./distance_nearest_vehicle_3);

%Assign to workspace some useful 'global' quantities
assignin('base','delta_diff',delta_diff);
assignin('base','Td_diff',Td_diff);
assignin('base','heading_error',heading_error);
assignin('base','lateral_error',lateral_error);
assignin('base','speed_error',speed_error);
assignin('base','proximity',proximity);
assignin('base','distance_nearest_vehicle_1',distance_nearest_vehicle_1);
assignin('base','distance_nearest_vehicle_2',distance_nearest_vehicle_2);
assignin('base','distance_nearest_vehicle_3',distance_nearest_vehicle_3);


f           =   1e3*(delta_diff'*delta_diff)+ ...
                1e-5*(Td_diff'*Td_diff) + ...
                1e2*(heading_error'*heading_error) + ...
                6e-1*(lateral_error'*lateral_error) + ...
                5e-2*(speed_error'*speed_error) + ...
                10* proximity;


%% Stack cost and constraints
v           =   [f;h];


