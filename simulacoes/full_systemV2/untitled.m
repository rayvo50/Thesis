% % MEMES



% % Sample data - replace these with your actual data
% times = [1, 2, 3, 4, 5]; % Time stamps
% estimates = [30, 45, 60, 55, 50]; % Orientation estimates in degrees
% covariances = [4, 3, 5, 2, 4]; % Covariance values
% 
% % Calculate confidence intervals
% sigma = sqrt(covariances);
% lower_bounds = estimates - 2 * sigma;
% upper_bounds = estimates + 2 * sigma;
% 
% % Plotting
% figure; hold on;
% plot(times, estimates, '-o', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Estimated Orientation');
% fill([times, fliplr(times)], [lower_bounds, fliplr(upper_bounds)], 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% Confidence Interval');
% xlabel('Time');
% ylabel('Orientation (degrees)');
% title('Orientation Estimate Over Time with Confidence Intervals');
% legend show;
% grid on;
% hold off;



% % Load the image
% [img, map, alpha] = imread('AUVs.png');  % Ensure 'robot.png' is in your MATLAB path
% 
% % Resize the image (make it smaller)
% scaleFactor = 1;  % Adjust scaleFactor to get a suitable size
% %img = imresize(img, scaleFactor);
% %alpha = imresize(alpha, scaleFactor);
% 
% % Example trajectory and yaw angles
% x = linspace(0, 10, 100);
% y = 10 * sin(0.5 * x);
% theta = linspace(0, 2 * pi, 100);  % Example yaw angles
% 
% % Define the actual size you want the image to cover in data units
% actualWidth = 3; % Width in data units (e.g., meters)
% actualHeight = 0.6; % Height in data units
% 
% % Calculate scaling factors
% xScale = actualWidth / 42; % Scale factor for width
% yScale = actualHeight / 7; % Scale factor for height
% 
% figure;
% ax = gca;
% hold on;
% axis equal;
% grid on;
% 
% % Set the axis limits such that they make sense for your trajectory coordinates
% 
% % Now, plot the trajectory and robot image
% plot(ax, x, y, 'b-', 'LineWidth', 1.5);
% 
% % Update plot in a loop (if you want to animate)
% for i = 1:10:length(x)
%     clf;  % Clear figure to update position
%     hold on;
%     plot(x, y, 'b-', 'LineWidth', 1.5);
%     ax = gca;
%     plotRotatedImage(ax, x(i), y(i), theta(i), img, alpha, xScale, yScale);
%     pause(1);
%     hold off;
% end
% 
% 
% 
% % Define a function to rotate and plot the image
% function plotRotatedImage(ax, x, y, theta, img, alpha, xScale, yScale)
%     % Rotate the image
%     rotatedImg = imrotate(img, rad2deg(-theta), 'bilinear', 'crop');
%     rotatedAlpha = imrotate(alpha, rad2deg(-theta), 'bilinear', 'crop');
% 
%     % Define the plot coordinates where the image should be displayed
%     [imgHeight, imgWidth, ~] = size(rotatedImg);
% 
%     % Adjust x and y coordinates based on the scaling factors
%     xLimits = x + (imgWidth * [-xScale / 2, xScale / 2]);
%     yLimits = y + (imgHeight * [-yScale / 2, yScale / 2]);
% 
%     % Display the image with the correct plot coordinates
%     image('CData', rotatedImg, 'XData', xLimits, 'YData', yLimits, ...
%           'AlphaData', rotatedAlpha, 'Parent', ax);
% end
% 
% 
% 
% 







% 
% % Example trajectory and yaw angles
% x = linspace(0, 10, 100);
% y = 10 * sin(0.5 * x);
% theta = linspace(0, 2 * pi, 100);  % Example yaw angles
% 
% % Create a figure
% figure;
% hold on;
% axis equal;
% grid on;
% 
% plot(x, y, 'b-', 'LineWidth', 1.5);  % Blue line for trajectory
% xlabel('X Position');
% ylabel('Y Position');
% title('Robot Trajectory and Orientation');
% 
% 
% % Triangle representing the robot
% baseTriangle = [0 -1; 1 0.5; -1 0.5];  % Basic triangle vertices
% triangleSize = 0.5;  % Scaling factor for the triangle size
% 
% for i = 1:10:length(x)  % Update every 10 steps for clarity
%     % Calculate the rotation matrix
%     R = [cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))];
% 
%     % Rotate and scale the triangle
%     rotatedTriangle = (R * baseTriangle')' * triangleSize;
% 
%     % Translate the triangle to the current position
%     translatedTriangle = bsxfun(@plus, rotatedTriangle, [x(i), y(i)]);
% 
%     % Plot the triangle
%     fill(translatedTriangle(:,1), translatedTriangle(:,2), 'r');
% end
% axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1]);























% 
% % Define the grid
% [x, y] = meshgrid(linspace(-10, 10, 100), linspace(-10, 10, 100));
% 
% % Initial mean and covariance
% mean0 = [0 0];
% cov0 = [1 0; 0 1];
% 
% % Total frames in the animation
% numFrames = 50;
% 
% % Create the figure
% figure;
% 
% 
% % Initialize arrays to store means and covariances
% means = zeros(numFrames, 2);
% covariances = zeros(2, 2, numFrames);
% 
% for i = 1:numFrames
%     % Change mean linearly
%     means(i, :) = mean0 + [0.2*i, 0.1*i];
% 
%     % Change covariance over time
%     covariances(:, :, i) = cov0 + [0.05*i 0; 0 0.05*i];
% end
% 
% for i = 1:numFrames
%     % Current mean and covariance
%     mu = means(i, :);
%     Sigma = covariances(:, :, i);
% 
%     % Calculate the Gaussian function
%     Z = mvnpdf([x(:) y(:)], mu, Sigma);
%     Z = reshape(Z, size(x));
% 
%     % Clear current figure
%     clf;
% 
%     % Create contour plot of the current Gaussian
%     contour(x, y, Z);
%     xlabel('X');
%     ylabel('Y');
%     title(sprintf('Frame %d: Mean = (%.2f, %.2f), Covariance = [%0.2f, %0.2f; %0.2f, %0.2f]', i, mu(1), mu(2), Sigma(1,1), Sigma(1,2), Sigma(2,1), Sigma(2,2)));
% 
%     % Pause for a bit before next frame
%     pause(0.1);
% end
% 
