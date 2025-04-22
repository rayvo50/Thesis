% Simulation parameters
total_time = 100;    % Total simulation time in seconds
dt = 0.1;            % Time step in seconds
v = 1;               % Constant speed of the AUV
time = 0:dt:total_time; % Time array

% Sinusoidal wave parameters
A = 1;               % Amplitude of the sinusoidal path
omega = 1;           % Frequency of the sinusoidal path

% Initialize arrays
x_position = zeros(size(time));
y_position = zeros(size(time));
velocity_x = zeros(size(time));
velocity_y = zeros(size(time));

% Initial conditions
y_position(1) = 0;   % Initial y position
x_position(1) = A * sin(omega * y_position(1)); % Initial x position

% Compute the trajectory and velocities
for i = 2:length(time)
    % Approximate the next y position incrementally
    delta_y = v * dt / sqrt(1 + (A * omega * cos(omega * y_position(i-1)))^2);
    y_position(i) = y_position(i-1) + delta_y;
    x_position(i) = A * sin(omega * y_position(i));

    % Compute the velocity components
    velocity_x(i) = (x_position(i) - x_position(i-1)) / dt;
    velocity_y(i) = (y_position(i) - y_position(i-1)) / dt;
end

% Calculate yaw (heading) and yaw rate
yaw = atan2(velocity_y, velocity_x);
yaw_rate = [0, diff(yaw) / dt];

% Combine the data into arrays
trajectory = [time' x_position' y_position'];
velocity_inertial = [time' velocity_x' velocity_y'];
yaw_data = [time' yaw'];
yaw_rate_data = [time' yaw_rate'];

% Rotate velocities to the body frame
velocity_body = zeros(size(velocity_inertial));
for i = 1:length(time)
    R = [cos(yaw(i)), -sin(yaw(i)); sin(yaw(i)), cos(yaw(i))];
    velocity_body(i, 2:3) = (R * velocity_inertial(i, 2:3)')';
end
velocity_body(:,1) = time';
