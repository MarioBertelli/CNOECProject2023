function envVisualization(x, y, x_ego, y_ego, theta_ego, C_proximity, C_dist, car_width, car_length, y_lanes)
    % ENVVISUALIZATION - Create an animated visualization of an environment with vehicles.
    % Input:
    %   x: Matrix of x-coordinates of other vehicles at different time steps (MxN), where M is the number of time steps, and N is the number of vehicles.
    %   y: Matrix of y-coordinates of other vehicles at different time steps (MxN), corresponding to the x-coordinates.
    %   x_ego: Vector of x-coordinates of the ego vehicle at different time steps (Mx1).
    %   y_ego: Vector of y-coordinates of the ego vehicle at different time steps (Mx1).
    %   theta_ego: Vector of ego vehicle orientations in radians at different time steps (Mx1).
    %   C_proximity: Covariance matrix to differentiate lateral and longitudinal proximity to other vehicles (Cost function entry) (2x2)
    %   C_dist: Covariance matrix to differentiate lateral and longitudinal distance to other vehicles (Constraint purpose) (2x2)
    %   car_width: Car width in meters [1x1]
    %   car_length: Car length in meters [1x1]
    %   y_lanes: column vectro of the possible lane references [3x1]
   
    %% Visualized Environment Settings
    % A bit of magic
    x_view_before_after = 25;
    % Track structure info
    centerlines = y_lanes;
    track_width = 3;
    y_limit_min = - 2;
    y_limit_max = 10;
    limit_draw_height = 2;

    %% Safety ellipse parameters
    % major axis
    a = C_dist(2,2);       
    % minor axis 
    b = C_dist(1,1);
    % Angles of definition of the ellipse
    th = linspace(0,2*pi); 
    % Ego vehicle ellipse points coordinates init
    xe_ego=zeros(1,100);
    ye_ego=zeros(1,100);
    % Other vehicles ellipse points coordinates init
    xe=zeros(length(x),100);
    ye=zeros(length(y),100);

    %% Video Setup
    % Set up video file parameters
    outputVideo = VideoWriter('testAnimation.mp4');
    outputVideo.FrameRate = 10; % Adjust the frame rate as needed
    open(outputVideo);

    % Set the initial axis limits to fit your needs
    x_min = min([x(:); x_ego]) - car_length;
    x_max = max([x(:); x_ego]) + car_length;
    y_min = min([y(:); y_ego; (y_limit_min-limit_draw_height + car_width)]) - car_width;
    y_max = max([y(:); y_ego; (y_limit_max+limit_draw_height - car_width)]) + car_width;

    % Create a figure and set the axis limits
    fig = figure;

    % Increase the figure size and resolution
    set(fig, 'Position', [0, 0, 1500, 200]); % Adjust the size as needed
    set(fig, 'PaperPositionMode', 'auto');
    set(fig, 'Renderer', 'Painters'); % Use the 'Painters' renderer for vector graphics

    xlim([x_min - x_view_before_after, x_min + x_view_before_after]);
    ylim([y_min, y_max]);
    axis equal;

    %% Animation
    % Begin creating the animation
    sample_skip = 100;
    for t = 1:sample_skip:size(x, 1)

        % Calculate distances between ego vehicle and all other vehicles
        distances = sqrt((x_ego(t) - x(t, :)).^2 + (y_ego(t) - y(t, :)).^2);

        % Sort vehicles by distance
        [~, sorted_indices] = sort(distances);

        % Select the three nearest vehicles
        nearest_indices = sorted_indices(1:3);

        % Clear the figure
        clf;

        % Set the axis limits to maintain fixed dimensions
        axis equal;
        
        title('Environment Visualization'),xlabel('xSim [m]'),ylabel('ySim [m]');
        % Uncomment this for full street view
        xlim([x_min, x_max]);
        % Uncomment this for vehicle eye bird view
        %xlim([x_ego(t) - x_view_before_after, x_ego(t) + x_view_before_after]);
        ylim([y_min, y_max]);

        % Draw lanes or other environmental elements
        for i = 1:length(centerlines)
            line([x_min, x_max], [centerlines(i) - track_width/2, centerlines(i)- track_width/2], 'LineStyle', '--', 'Color', 'k', 'LineWidth', 1.0);
            line([x_min, x_max], [centerlines(i) + track_width/2, centerlines(i)+ track_width/2], 'LineStyle', '--', 'Color', 'k', 'LineWidth', 1.0);
        end
        line([x_min, x_max], [centerlines(1) - track_width/2, centerlines(1)- track_width/2], 'LineStyle', '-', 'Color', 'k', 'LineWidth', 1.5);
        line([x_min, x_max], [centerlines(end) + track_width/2, centerlines(end) + track_width/2], 'LineStyle', '-', 'Color', 'k', 'LineWidth', 1.5);
        rectangle('Position', [x_min, y_limit_min - limit_draw_height, x_max - x_min, limit_draw_height], 'EdgeColor', 'k', 'FaceColor', 'k');
        rectangle('Position', [x_min, y_limit_max, x_max - x_min, limit_draw_height], 'EdgeColor', 'k', 'FaceColor', 'k');

        % Draw vehicles as rectangles and add numbers
        hold on;
        
        %Safety ellipse around vehicles
        for i=1:size(x, 2)
            xe(i,:) = x(t,i)+a*cos(th); 
            ye(i,:) = y(t,i)+b*sin(th);
        end

        %Plot other vehicles
        for i = 1:size(x, 2)
            rectangle('Position', [x(t, i) - car_length/2, y(t, i) - car_width/2, car_length, car_width], 'EdgeColor', 'r', 'FaceColor', 'r');
            text(x(t, i), y(t, i), num2str(i), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'w');
            plot(xe(i,:), ye(i,:), 'r');
        end

        % Draw the ego vehicle as a blue rectangle
        % Create a polyshape for the ego vehicle
        ego_x = x_ego(t) + [-car_length/2, car_length/2, car_length/2, -car_length/2];
        ego_y = y_ego(t) + [-car_width/2, -car_width/2, car_width/2, car_width/2];
        ego_shape = polyshape(ego_x, ego_y);

        %Safety distance ellipse around ego vehicle
        xe_ego(1,:) = x_ego(t)+a*cos(th); 
        ye_ego(1,:) = y_ego(t)+b*sin(th);
        % Ego vehicle ellipse rototraslation with body
        ERot= zeros(2,100);
        for i=1:100
            ERot(:,i)=[cos(theta_ego(t)) -sin(theta_ego(t)); sin(theta_ego(t)) cos(theta_ego(t))] * [xe_ego(1,i)-x_ego(t) ye_ego(1,i)-y_ego(t)]' + [x_ego(t) y_ego(t)]';
        end
        % Rotate the ego vehicle
        ego_shape = rotate(ego_shape, theta_ego(t) * 360 / 2.0 / pi, [x_ego(t), y_ego(t)]);
        
        % Plot the ego vehicle
        plot(ego_shape, 'FaceColor', 'b', 'EdgeColor', 'b');
        text(x_ego(t), y_ego(t), 'Ego', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'w');
        %Plot ellipse around ego vehigle, properly rotated
        
        plot(ERot(1,:), ERot(2,:),'b');

        % Draw dashed lines between ego vehicle and three nearest vehicles
        for i = 1:3
            line([x_ego(t), x(t, nearest_indices(i))], [y_ego(t), y(t, nearest_indices(i))], 'LineStyle', ':', 'Color', [0.2, 0.2, 0.2]);
        end

        hold off;

        % Add any indicators or annotations for constraints or bounds

        % Update the visualization
        drawnow;

        % Capture the frame and add it to the video
        writeVideo(outputVideo, getframe(gcf));

        % You can also add a delay to control the animation speed
        pause(0.01); % Set the desired delay in seconds

    end

    % Close the video file
    close(outputVideo);
end
