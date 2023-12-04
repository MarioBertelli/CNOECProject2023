function debugVisualization(timeScale, delta_diff, Td_diff, heading_err, lateral_err, speed_err, proximity)
% DEBUGVISUALIZATION - Create an animated visualization of useful quantities for debugging purposes.
    % Input:
    %     timeScale         = timeScale as abscissa in plotting
    %     delta_diff        = Steering first derivative
    %     Td_diff           = Torque first derivative
    %     heading_err       = Orientation error with respect to to horizontal direction
    %     lateral_err       = Lateral position error with respect to lane reference
    %     speed_err         = Speed error with respect to setpoint
    %     proximity         = Proximity to others vehicles
debugFig1=figure(3);
subplot(3,3,1);
plot(Np,delta_diff);


end