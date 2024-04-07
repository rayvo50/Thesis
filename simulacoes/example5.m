clearvars;clc;close all; format longg;

scenario = 2;

% noises standard deviation
range_noise = 0.5;
velocity_noise = 0.1;
Dt= 0.1;

if scenario == 1
    % trajetoria circular
    Pd = [20,10]; v_c = [0.1,0];
    u = 0.5; r = -0.02;
    Dt= 0.1; t = 0:Dt:2*pi/-r;
    vx = u*cos(90 +r*t)+ v_c(1); vy = u*sin(90+r*t)+v_c(2);
    P0 = [u/r,0];
    Px = P0(1) + cumsum(vx)*Dt; Py = P0(2) + cumsum(vy)*Dt;
    vx_m = vx-v_c(1) + velocity_noise*randn(size(vx));
    vy_m = vy-v_c(2) + velocity_noise*randn(size(vy));
    r_m = sqrt( (Px-Pd(1)).^2 + (Py -Pd(2)).^2 ) + range_noise*randn(size(Px));
elseif scenario == 2  
    % trajetoria sinusoidal
    Pd = [0,0]; v_c = [0,0];
    t = 0:Dt:300;
    vx = zeros(size(t))+0.1+v_c(1); 
    vy = 2*sin(0.05*t)+0.1+v_c(2);
    P0 = [-100,-100];
    Px = P0(1) + cumsum(vx)*Dt;
    Py = P0(2) + cumsum(vy)*Dt;
    vx_m = vx-v_c(1) + velocity_noise*randn(size(vx));
    vy_m = vy-v_c(2) + velocity_noise*randn(size(vy));
    r_m = sqrt( (Px-Pd(1)).^2 + (Py -Pd(2)).^2 ) + range_noise*randn(size(Px));
elseif scenario == 3 
    % trajetoria lawnmower
    Pd = [0,0]; v_c = [0.2,0];
    u = 0.5; r = 0.1;
    %todo
end



%% simulate filter
% system model
A = [1,0,Dt,0;0,1,0,Dt;0,0,1,0;0,0,0,1]; B = [Dt,0;0,Dt;0,0;0,0]; G = Dt*eye(4);

% filter design
R = 500; 
Q = [1,0,0,0; ...
    0,1,0,0; ... 
    0,0,0.001,0;
    0,0,0,0.001];

% filter initialization
x = zeros(4,length(t));
x(:,1) = [P0(1),P0(2),0,0];
e = zeros(size(t));
K = zeros(4,3324);
P = Q;
u = [vx_m;vy_m];
y = r_m;
for k=1:length(t)
    % predict step
    x_ = A*x(:,k) + B*u(:,k);
    P_ = A*P*A' + Q;
    
    % filter step
    H = compute_H(x_, Pd);
    K(:,k) = (H*P_*H' + R)\(P*H');
    e(k) = y(k)-h(x_,Pd);
    x(:,k+1) = x_ + K(:,k)*e(k);
    P = (eye(4) - K(:,k)*H)*P_;
end

% plots
figure; hold on; grid on;
plot(Py, Px, 'LineWidth', 2); 
plot(x(2,:), x(1,:), 'LineWidth', 2); 
plot(Pd(2), Pd(1), '^', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
legend('Ground truth', 'Filter Prediction', 'Dock location', 'Location', 'best'); xlabel('East y [m]'); ylabel('North y [m]');


figure; plot(x(3,:)); hold on; plot(x(4,:));



function y_p = h(x, P_d)
    y_p = sqrt((x(1)-P_d(1)).^2 + (x(2)-P_d(2)).^2);
end

function H = compute_H(x_hat, P_d)
    H = zeros(1,4);
    denominator = sqrt((x_hat(1)-P_d(1))^2+(x_hat(2)-P_d(2))^2);
    H(1) = (x_hat(1)-P_d(1))/denominator;
    H(2) = (x_hat(2)-P_d(2))/denominator;
end