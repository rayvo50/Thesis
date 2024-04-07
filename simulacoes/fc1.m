clearvars
close all
% Parameters for synthetic data generation
duration = 20;          % Total simulation time in seconds
sampleRate = 100;       % Sampling rate in Hz
dt = 1/sampleRate;      % Time step
t = (0:dt:duration-dt)'; % Time vector


Kp = 1;
Ki = 0.1;
gyro_bias = 5/180*pi;
gyro_noise = 0.1;
accel_noise = 0.1;

s = tf('s'); % Laplace variable s
p = 0.05;
lpf = p/(p + s);
hpf = s/(p + s);
figure; bode(lpf); hold on; bode(hpf); hold off; grid on;

%Sinusoidal yaw parameters
amplitude = pi/4;       % Amplitude of the yaw variation (radians)
frequency = 0.1;        % Frequency of the yaw variation (Hz)

% Generate sinusoidal yaw angle
yawAngle = amplitude * sin(2 * pi * frequency * t);

% Generate synthetic gyro data (derivative of yaw angle)
gyroData = [t, [0; diff(yawAngle) / dt] + gyro_bias + gyro_noise * randn(length(t), 1)]; % Adding noise

% Assuming the accelerometer provides angle measurements directly (simplified)
% In reality, accelerometer measurements would be processed to estimate tilt
accelData = [t, yawAngle + accel_noise * randn(length(t), 1)]; % Adding noise
% Store the data in the MATLAB base workspace for Simulink
assignin('base', 'gyroData', gyroData);
assignin('base', 'accelData', accelData);

% Run the Simulink model
modelName = 'filtros'; 
simOut = sim(modelName);

% Extract the filter output from the simulation results
filterOutput = simOut.get('filterOutput'); % Ensure this matches your To Workspace block variable name

% Compute gyro dead-reckoning (integrate the gyro measurements over time)
gyroDeadReckoning = cumsum(gyroData(:,2) * dt);

% Plot the results
figure;
plot(t, accelData(:,2), 'DisplayName', 'Accelerometer (tilt angle)');
hold on;
plot(t, gyroDeadReckoning, 'DisplayName', 'Gyro Dead-Reckoning (integrated)');
plot(filterOutput, 'DisplayName', 'Complementary Filter Output');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Measurements vs. Filter Output vs. Gyro Dead-Reckoning');
legend;
grid on;
