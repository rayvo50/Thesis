Pd = [100,120,170];
P = [0,0,0];
Delta =10;

m1 = tand(Pd(3));
b1 = Pd(2) - m1*Pd(1);

m2 = tand(Pd(3)+90);
b2 = P(2) - m2*P(1);

x = (b2-b1)/(m1-m2);
y = m1*x +b1;

x_los = x + Delta*cosd(Pd(3)+180);
y_los = y + Delta*sind(Pd(3)+180);

yaw_des = atan2d(y_los-P(2),x_los-P(1));
m3 = tand(yaw_des);
b3 = P(2) - m3*P(1);


% Plot the line
xLine = linspace(Pd(1)-100, Pd(1), 400); % Generate points for the line
yLine = m1*xLine + b1; % Calculate corresponding y values based on the line equation
plot(yLine, xLine, 'b-', 'LineWidth', 2); % Plot the line
hold on;

% Plot the original point
plot(P(2), P(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Plot the original point

% Plot the closest point on the line
plot(y, x, 'g*', 'MarkerSize', 10, 'LineWidth', 2); % Plot the closest point
% Plot the closest point on the line
plot(y_los, x_los, 'r*', 'MarkerSize', 10, 'LineWidth', 2); % Plot the closest point

plot(Pd(2), Pd(1), '^', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'green', 'MarkerSize', 10);

xLine = linspace(P(1), P(1)+20, 400); % Generate points for the line
yLine = m3*xLine + b3; % Calculate corresponding y values based on the line equation
plot(yLine, xLine, 'b-', 'LineWidth', 2); % Plot the line
hold on;

% Enhancements for visualization
grid on;
axis equal;
xlabel('X');
ylabel('Y');
legend('Path', 'AUV', 'Closest Point', 'Target', 'Dock', 'yaw_desired', 'Location', 'Best');
title('Closest Point on Line to a Given Point');

% Display the coordinates of the closest point
fprintf('Closest point on the line to (x0, y0): (%.2f, %.2f)\n', x, y);

hold off;
