function envVisualization(x, y, x_ego, y_ego, theta_ego)
    % Vehicle dimensions
    car_width = 1.5;
    car_length = 3;

    % Set up video file parameters
    outputVideo = VideoWriter('testAnimation.mp4');
    outputVideo.FrameRate = 30; % Adjust the frame rate as needed
    open(outputVideo);

    % Set the initial axis limits to fit your needs
    x_min = min([x(:); x_ego]) - car_length;
    x_max = max([x(:); x_ego]) + car_length;
    y_min = min([y(:); y_ego]) - car_width;
    y_max = max([y(:); y_ego]) + car_width;

    % Create a figure and set the axis limits
    fig = figure;
    % Increase the figure size and resolution
    set(fig, 'Position', [100, 100, 1200, 800]); % Adjust the size as needed
    set(fig, 'PaperPositionMode', 'auto');
    set(fig, 'Renderer', 'Painters'); % Use the 'Painters' renderer for vector graphics

    xlim([x_min, x_max]);
    ylim([y_min, y_max]);
    axis equal;

    % Add other customizations to the visualization environment here

    % Begin creating the animation
    for t = 1:size(x, 1)
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
        xlim([x_min, x_max]);
        ylim([y_min, y_max]);

        % Draw lanes or other environmental elements if necessary

        % Draw vehicles as rectangles and add numbers
        hold on;
        for i = 1:size(x, 2)
            rectangle('Position', [x(t, i) - car_length/2, y(t, i) - car_width/2, car_length, car_width], 'EdgeColor', 'r', 'FaceColor', 'r');
            text(x(t, i), y(t, i), num2str(i), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'w');
        end

        % Draw the ego vehicle as a blue rectangle
        % Create a polyshape for the ego vehicle
        ego_x = x_ego(t) + [-car_length/2, car_length/2, car_length/2, -car_length/2];
        ego_y = y_ego(t) + [-car_width/2, -car_width/2, car_width/2, car_width/2];
        ego_shape = polyshape(ego_x, ego_y);

        % Rotate the ego vehicle
        ego_shape = rotate(ego_shape, theta_ego(t) * 360 / 2.0 / pi, [x_ego(t), y_ego(t)]);

        % Plot the ego vehicle
        plot(ego_shape, 'FaceColor', 'b', 'EdgeColor', 'b');
        text(x_ego(t), y_ego(t), 'Ego', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'w');

        % Draw dashed lines between ego vehicle and three nearest vehicles
        for i = 1:3
            line([x_ego(t), x(t, nearest_indices(i))], [y_ego(t), y(t, nearest_indices(i))], 'LineStyle', '--', 'Color', 'k');
        end

        hold off;

        % Add any indicators or annotations for constraints or bounds

        % Update the visualization
        drawnow;

        % Capture the frame and add it to the video
        writeVideo(outputVideo, getframe(gcf));

        % You can also add a delay to control the animation speed
        pause(0.03); % Set the desired delay in seconds
    end

    % Close the video file
    close(outputVideo);
end
