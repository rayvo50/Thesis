% preamble
clearvars;clc;close all;format longg;

% initial conditions and parameters
Pd = [20,20,0];
P0 = [0,0,0];
r_max = 0.5;
u_rate = 0.1;

%% simulate filter
% filter design
R = 100*eye(2); 
Q = [1,0,0,0,0; ...
    0,1,0,0,0; ...
    0,0,0.01,0,0;...
    0,0,0,0.02,0;...
    0,0,0,0,0.02];

% initializations
x = zeros(4,length(t));
x(:,1) = [P0(1),P0(2),0,0]';
ksi = zeros(8,length(t));
y0 = measure(x(:,1));
ksi(:,1) = [y0;0;0;0;0];

P = Q;
y = [Px_m;Py_m];
for k=1:length(t)
    % read sensors
    y = measure(x);

    % filter
    % predict step
    x_ = f(x(:,k),Dt);
    F = compute_F(x(:,k),Dt);
    P_ = F*P*F' + Q;
    % filter step
    K = (P*C')*inv(C*P_*C' + R);
    e(:,k) = y(:,k)-C*x_;
    x(:,k+1) = x_ + K*e(:,k);
    P = (eye(5) - K*C)*P_;
    
    %compute control

    %aply control to the plant

end

% plots
figure; hold on; 
plot(Py_m, Px_m); 
plot(Py, Px, 'LineWidth', 2); 
plot(x(2,:), x(1,:), 'LineWidth', 2); 
grid on;
legend('Measurements','Ground truth', 'Filter Prediction', 'Dock location', 'Location', 'best'); 
xlabel('East y [m]'); ylabel('North y [m]');


