% clear all; 
% close all;
% % Define y_lanes and generate a linspace for actual_y values
% y_lanes = [2, 5, 8];
% actual_y_values = linspace(0, 10, 1000); % Adjust the range as needed
% 
% % Initialize arrays to store penalties
% % Calculate error values and penalties for each actual_y value
% errors = zeros(size(actual_y_values));
% for i = 1:length(actual_y_values)
%     [errors(i)] = lateral_error_computation(y_lanes, actual_y_values(i));
% end
% 
% % Plot the lateral error values and penalties
% figure;
% 
% % Plot the lateral error values
% plot(actual_y_values, errors);
% xlabel('Actual y');
% ylabel('Lateral Error');
% title('Lateral Error for Different Actual y Values');

function [error] = lateral_error_computation(y_lanes, actual_y)
    error_left = (y_lanes(1) - actual_y);
    error_center = (y_lanes(2) - actual_y);
    error_right = (y_lanes(3) - actual_y);
    error_left  = error_left .* error_left;
    error_center  = error_center .* error_center;
    error_right  = error_right .* error_right;

    min_error = differentiable_min([error_left, error_center, error_right]);

    error = min_error;
end

function min_val = differentiable_min(errors)
    min_val = sum(errors .* softmin(errors));
end

function softmin_vals = softmin(errors)
    softmin_vals = exp(-errors) / sum(exp(-errors));
end
