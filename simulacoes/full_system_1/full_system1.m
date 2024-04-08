% preamble
clearvars;clc;close all;format longg;

% initial conditions and parameters
Pd = [100,100,180];
P0 = [0,0,0];
Dt=0.1;


% filter design
R = 1*eye(7);
Q = [1,   0.1, 0,  0,  0,   0,   0,   0;
     0.1, 1,   0,  0,  0,   0,   0,   0;
     0,   0,   1,  0,  0,   0,   0,   0;
     0,   0,   0,  1,  0,   0,   0,   0;
     0,   0,   0,  0,  0.1, 0,   0,   0;
     0,   0,   0,  0,  0,   1,   0.1, 0;
     0,   0,   0,  0,  0,   0.1, 1,   0;
     0,   0,   0,  0,  0,   0,   0,   1];
P = [1,   0.1, 0,  0,  0,   0,   0,   0;
     0.1, 1,   0,  0,  0,   0,   0,   0;
     0,   0,   1,  0,  0,   0,   0,   0;
     0,   0,   0,  1,  0,   0,   0,   0;
     0,   0,   0,  0,  0.1, 0,   0,   0;
     0,   0,   0,  0,  0,   100,   10, 0;
     0,   0,   0,  0,  0,   10, 100,   0;
     0,   0,   0,  0,  0,   0,   0,   1];




%% simulation
% initializations
t = 0:Dt:20;
x = zeros(5,length(t));
x(:,1) = [P0(1),P0(2),0,P0(3),0]';
tau = zeros(2, length(t));

chi = zeros(8,length(t));
y0 = measure(x(:,1),Pd);
chi(:,1) = [0;0;0;0;0;1;1;0];
y = zeros(7,length(t));
e = zeros(size(y));
temp = zeros(size(y));
for k=2:length(t)
    % read sensors
    y_ = measure(x(:,k-1),Pd);
    u = y_(end);
    y(:,k) = y_(1:end-1);

    % filter
    % predict step
    chi_ = f(chi(:,k-1),u,Dt);
    F = compute_F(chi(:,k),Dt);
    P_ = F*P*F' + Q;
    % filter step
    H = compute_H(chi_);
    K = (P*H')*inv(H*P_*H' + R);
    temp(:,k) = h(chi_);
    e(:,k) = y(:,k)-h(chi_);
    chi(:,k) = chi_ + K*e(:,k);
    P = (eye(8) - K*H)*P_;
    
    %compute control
    tau(:,k) = controller(x(:,k), Pd);

    %aply control to the plant
    x(:,k) = model(x(:,k-1), tau(:,k), Dt);
end

% plots
figure; hold on; grid on;
plot(y(2,:), y(1,:));
plot(x(2,:), x(1,:), 'LineWidth', 2); 
plot(chi(2,:), chi(1,:), '*', 'LineWidth', 2); 
plot(Pd(2), Pd(1), '^', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
legend('Measurements','Ground truth', 'Filter Prediction', 'Dock location', 'Location', 'best'); 
xlabel('East y [m]'); ylabel('North y [m]');
figure; hold on; grid on;
plot(chi(7,:),chi(6,:),'*')
figure; hold on; grid on;
plot(chi(8,:))


