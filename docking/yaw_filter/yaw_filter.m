clearvars;clc;close all;
load('yaw_data.mat');

% generate measuremtns 
gyro_noise = 0.5;
gyro_bias = 5;
gyro_meas = yaw_rate + gyro_bias + gyro_noise*randn(size(yaw_rate));
compass_noise = 10;
compass_meas = yaw + compass_noise*randn(size(yaw));
%plot(compass_meas);figure;plot(gyro_meas);

% system model
Dt = 0.1;
A = [1,Dt;0,1]; B = [Dt;0]; G = [Dt,0;0,Dt]; C = [1,0];
R = 10; Q = [0.1,0;0,0.1];

% filter initialization
time = 1:497;
x = zeros(2,length(time));
P = Q;
u = gyro_meas;
y = compass_meas;
for k=1:496
    %one step ahead prediction
    K = P*C'*inv(C*P*C' + R);
    x(:,k+1) = A*(eye(2) - K*C)*x(:,k) + B*u(k) + A*K*y(k);
    P = A*P*A' - A*K*C*P*A' + G*Q*G';
end
yaw_filtered = x(1,:);
plot(compass_meas); hold on; plot(yaw_filtered);