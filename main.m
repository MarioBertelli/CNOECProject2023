close all
clear all
clc

% Plant model parameters initialization
run('simPlannerInit.m');
run('plantModelInit.m');

%Call function that gives x and y coordinate matrices of the N vehicles
%based on previous settings
[x, y] = simPlanner(x0, lane0, vx, tSwitch, laneSwitch, N, num_time_steps, Ts_simulation);

% Dynamical model resolution with ode45
tic
for ind=2:N_o45
    zout_temp           =   ode45(@(t,z)vehicle(t,z,uout_o45(:,ind-1),0,th),[0 Ts_o45],zout_o45(:,ind-1));
    zout_o45(:,ind)     =   zout_temp.y(:,end);
    [~,F]               =   vehicle(0,zout_o45(:,ind),uout_o45(:,ind-1),0,th);
    Fout_o45(:,ind)     =   F;
    % uout_o45(:,ind)     =   uout_o45(:,1);
end
t_o45=toc;

% Ego vehicle states association
x_ego=zout_o45(1,:)';
y_ego=zout_o45(2,:)';
ego_orientation=zout_o45(5,:)';

% Call the envVisualization function with the sample data
envVisualization(x, y, x_ego, y_ego, ego_orientation, [10 20 30 40 50 60]);

% Plot ego vehicle states
% figure(1),plot(zout_o45(1,:),zout_o45(2,:)),grid on, hold on,xlabel('X (m)'),ylabel('Y (m)')
% figure(2),plot(tvec_o45,zout_o45(3,:)*3.6),grid on, hold on,xlabel('Time (s)'),ylabel('Longitudinal speed (km/h)')
% figure(3),plot(tvec_o45,zout_o45(4,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Sideslip angle (deg)')
% figure(4),plot(tvec_o45,zout_o45(5,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw angle (deg)')
% figure(5),plot(tvec_o45,zout_o45(6,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw rate (deg/s)')
% figure(6),plot(tvec_o45,Fout_o45(1,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Front lateral force (N)')
% figure(7),plot(tvec_o45,Fout_o45(2,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Rear lateral force (N)')
% figure(8),plot(tvec_o45,zout_o45(6,:).*zout_o45(3,:)/9.81),grid on, hold on,xlabel('Time (s)'),ylabel('Lateral acceleration (g)')
