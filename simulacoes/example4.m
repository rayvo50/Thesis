clearvars;clc;close all;format longg;

scenario = 1;

% noises standard deviation
position_noise = 1;
Dt= 0.1;

if scenario == 1
    % trajetoria circular
    u = 0.5; r = -0.02;
    Dt= 0.1; t = 0:Dt:1.9*pi/-r;
    vx = u*cos(90 +r*t); vy = u*sin(90+r*t);
    P0 = [u/r,0];
    Px = P0(1) + cumsum(vx)*Dt; Py = P0(2) + cumsum(vy)*Dt;
    
    Px_m = Px + position_noise*randn(size(Px));
    Py_m = Py + position_noise*randn(size(Py));

elseif scenario == 2  
    % trajetoria sinusoidal
    v_c = [0.2,0];
    t = 0:Dt:300;
    vx = zeros(size(t))+0.1+v_c(1); 
    vy = 2*sin(0.05*t)+0.1+v_c(2);
    P0 = [-100,-100];
    Px = P0(1) + cumsum(vx)*Dt;
    Py = P0(2) + cumsum(vy)*Dt;

    Px_m = Px + position_noise*randn(size(Px));
    Py_m = Py + position_noise*randn(size(Py));

elseif scenario == 3 
    % trajetoria lawnmower
    % TODO
end



%% simulate filter
% system model
C = [1,0,0,0,0;0,1,0,0,0]; G = Dt*eye(4);

% filter design
R = 100*eye(2); 
Q = [1,0,0,0,0; ...
    0,1,0,0,0; ... 
    0,0,0.01,0,0;...
    0,0,0,0.02,0;...
    0,0,0,0,0.02];

% filter initialization
x = zeros(5,length(t));
x(:,1) = [Px_m(1),Py_m(1),0,0,0]';
e = zeros(2,length(t));
P = Q;
y = [Px_m;Py_m];
for k=1:length(t)
    % predict step
    x_ = f(x(:,k),Dt);
    F = compute_F(x(:,k),Dt);
    P_ = F*P*F' + Q;
    
    % filter step
    K = (P*C')*inv(C*P_*C' + R);
    e(:,k) = y(:,k)-C*x_;
    x(:,k+1) = x_ + K*e(:,k);
    P = (eye(5) - K*C)*P_;
end

% plots
figure; hold on; 
plot(Py_m, Px_m); 
plot(Py, Px, 'LineWidth', 2); 
plot(x(2,:), x(1,:), 'LineWidth', 2); 
grid on;
legend('Measurements','Ground truth', 'Filter Prediction', 'Dock location', 'Location', 'best'); 
xlabel('East y [m]'); ylabel('North y [m]');


%figure; plot(x(3,:)); hold on; plot(x(4,:));



function x_p = f(x,Dt)
    x_p = zeros(5,1);
    x_p(1) = x(1,:) + x(3,:)*cos(x(4,:))*Dt;
    x_p(2) = x(2,:) + x(3,:)*sin(x(4,:))*Dt;
    x_p(3) = x(3,:);
    x_p(4) = x(4,:) + x(5,:)*Dt;
    x_p(5) = x(5,:);
end

function H = compute_F(x, Dt)
    H = eye(5);
    H(1,3) = cos(x(4,:))*Dt;
    H(2,3) = sin(x(4,:))*Dt;
    H(1,4) = -x(3,:)*sin(x(4,:))*Dt;
    H(2,4) = x(3,:)*cos(x(4,:))*Dt;
    H(4,5) = Dt;
end