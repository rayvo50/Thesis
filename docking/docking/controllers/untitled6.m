close all; clear; clc;

% --- Simulation Parameters ---
T = 60;           % total time [s]
Dt = 0.1;         % time step [s]
t = 0:Dt:T;       % time vector

% --- AUV Parameters (yaw) ---
mr = 4.14 + 0.5;  % added + rigid body inertia
Nr = 4.14;        % linear damping
Nrr = 6.23;       % quadratic damping

% --- Reference Yaw Signal ---
yaw_ref = pi/1.5* sin(0.2 * t);           % reference heading [rad]
yaw_rate_ref = diff(yaw_ref)/Dt ;  
yaw_acc_ref = diff(yaw_rate_ref)/Dt;  % second derivative
yaw_rate_ref = [yaw_rate_ref yaw_rate_ref(end)];% first derivative
yaw_acc_ref = [yaw_acc_ref yaw_acc_ref(end) yaw_acc_ref(end)];

% --- Initial States [yaw, yaw_rate] ---
x = [0; 0];  % [yaw; yaw_rate]

% --- Store Data for Plotting ---
yaw_log = zeros(size(t));
yaw_rate_log = zeros(size(t));
u_log = zeros(size(t));
debug = zeros(size(t));

% --- Controller Object ---
ctrl = yaw_SMC();  % your controller class (make sure it’s in path)

% --- Simulation Loop ---
for k = 1:length(t)
    % Extract ref
    yaw_d = yaw_ref(k);
    dyaw_d = yaw_rate_ref(k);
    ddyaw_d = yaw_acc_ref(k);

    % Compute control
    ctrl = ctrl.compute(x(1), x(2), yaw_d, dyaw_d, ddyaw_d, 0,0);  % zero surge/sway
    tau_r = ctrl.output;  % control output (torque)
    debug(k) = ctrl.debug;

    % --- AUV Yaw Dynamics ---
    yaw_ddot = (1/mr) * (tau_r - Nr * x(2) - Nrr * x(2)*abs(x(2)));
    x(2) = x(2) + yaw_ddot * Dt;  % yaw rate
    x(1) = x(1) + x(2) * Dt;      % yaw

    % Log
    yaw_log(k) = x(1);
    yaw_rate_log(k) = x(2);
    u_log(k) = tau_r;
end

% --- Plot ---
figure;
subplot(3,1,1)
plot(t, yaw_ref, '--', t, yaw_log, 'LineWidth', 2);
ylim([-1.5*pi,1.5*pi])
legend('Reference', 'Yaw');
ylabel('Yaw (rad)'); grid on;

subplot(3,1,2)
plot(t, yaw_rate_log, 'LineWidth', 2);

ylabel('Yaw Rate (rad/s)'); grid on;

subplot(3,1,3)
plot(t, u_log, 'LineWidth', 2);
%ylim([-10,10])
ylabel('Torque \tau_r (Nm)');
xlabel('Time (s)'); grid on;

figure()
hold on
plot(t, yaw_ref)
plot(t, yaw_rate_ref)
plot(t, yaw_acc_ref)


figure()
plot(t, debug)
