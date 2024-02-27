%%STANDARD IMAGE CREATOR FOR THE REPORT
fig=figure;
plot(delta_diff, 'b-','LineWidth',3);
title("Delta Diff",'Interpreter','latex');
xlabel('Space [m]','Interpreter','latex'); 
ylabel('Error [-]','Interpreter','latex');
legend("Delta Diff",'Interpreter','latex');
fontsize(fig,16,"points");

%CHANGE HERE IMAGE NAME TO BE SAVED (Locate the matlab path in project
%root, otherwise error)
%exportgraphics(fig,'C:\Users\Mario Bertelli\Documents\AAA_Mario\ProgettoCNOEC\REPORT\ReportExportedImages\lastTime.eps','Resolution',600,'BackgroundColor','white');
exportgraphics(fig,'REPORT\ReportExportedImages\ImageName.eps','Resolution',600,'BackgroundColor','white');