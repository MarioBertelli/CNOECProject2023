close all
%clear
clc
%% Print Script

hfig = figure;  % save the figure handle in a variable
fname = 'your figure';
% PS = PLOT_STANDARDS();

% ********************** Insert Your Plot Here **************************

% %plot(Xa_id,Xp_id,'b-','LineWidth',3);
% plot(Xa_id,Xp_id,Xa_id,Xp,'r-','LineWidth',3);
% hold on
% title('Ideal vs Actual Braking force distribution');
% %lgd1=legend('Ideal Xa vs Xp', 'Actual Xa vs Xp', 'Difference between Id and Actual');
% xlabel('Xa','Interpreter','latex');
% ylabel('Xp','Interpreter','latex');
% 
% figure;
% hold on
% for i=r2_vect
%     for j=k2_vect
%         plot(i,j, '.');
%     end
% end
% title('Feasable Design Variable Space','Interpreter','latex');
% %lgd1=legend();
% xlabel('$k_2$ [N/m]','Interpreter','latex');
% ylabel('$r_2$ [Ns/m]','Interpreter','latex');


% hold on
% for i=1:length(DV(:,1))
%     if(orderedObjMat(i,6)==0)
%         plot3(orderedObjMat(i,3),orderedObjMat(i,4), orderedObjMat(i,5),'rd');
%     else 
%         plot3(orderedObjMat(i,3),orderedObjMat(i,4), orderedObjMat(i,5),'b.');
%     end
% end
% title('Cost Function Values with Higlighted Pareto Set','Interpreter','latex');
% %lgd1=legend();
% xlabel('Discomfort [$m/s^2$]','Interpreter','latex');
% ylabel('Road Holding [N]','Interpreter','latex');
% zlabel('Working Space [m]','Interpreter','latex');
% grid on;



% hold on
% for i=1:length(DV(:,1))
%     if(orderedObjMat(i,6)==0)
%         plot(orderedObjMat(i,3),orderedObjMat(i,4),'rd');
%     else 
%         plot(orderedObjMat(i,3),orderedObjMat(i,4),'b.');
%     end
% end
% title('Cost Function Values with Higlighted Pareto Set','Interpreter','latex');
% %lgd1=legend();
% xlabel('Discomfort [$m/s^2$]','Interpreter','latex');
% ylabel('Road Holding [N]','Interpreter','latex');

% hold on;
% for i=1:length(DV(:,1))
%     if(orderedObjMat(i,6)==0)
%         plot(orderedObjMat(i,1),orderedObjMat(i,2),'rd');
%     else
%         plot(orderedObjMat(i,1),orderedObjMat(i,2),'b.');
%     end
% end
% title('Design Variables Values with Higlighted Pareto Set','Interpreter','latex');
% %lgd1=legend();
% xlabel('k2 [N/m]','Interpreter','latex');
% ylabel('r2 [Ns/m]','Interpreter','latex');

% hold on;
% for i=1:length(xWS)
%     plot(xWS(i,1), xWS(i,2), '.');
% end
% title('Pareto Optimal Design Variables Set with Weighted Sum');
% %lgd1=legend();
% xlabel('k2 [N/m]','Interpreter','latex');
% ylabel('r2 [Ns/m]','Interpreter','latex');

% plot(discSecWS,roadHSecWS,'LineWidth',3);
% title('Pareto Optimal Weighted Sum Cost Values Set');
% %lgd1=legend();
% xlabel('Discomfort [$m/s^2$]','Interpreter','latex');
% ylabel('Road Holding [N]','Interpreter','latex');

plot(sol1, sol2,'LineWidth',3);
title('Pareto Optimal Cost Values Set with Constrained Method','Interpreter','latex');
xlabel('Discomfort [$m/s^2$]','Interpreter','latex');
ylabel('Road Holding [N]','Interpreter','latex');
% *************************** End Of Plot *******************************

picturewidth = 30; % set this parameter and keep it forever
hw_ratio = 0.65; % feel free to play with this ratio
set(findall(hfig,'-property','FontSize'),'FontSize',22) % adjust fontsize to your document

% set(findall(hfig,'-property','Box'),'Box','off') % optional
set(findall(hfig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(hfig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(hfig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])

pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
% print(hfig,fname,'-dpdf','-painters','-fillpage')

% set the number of legend you want
%set(findall(lgd1, '-property', 'FontSize'), 'FontSize', 11)

set(gca,'TickLabelInterpreter','latex','FontSize',14);

print(hfig,fname,'-dpng','-painters')

savefig(hfig,'BeautifulPlot');

