clearvars;clc;close all;format longg;

scenario = 1;

% noises standard deviation
range_noise = 0.5;
bearing_noise = 5/180*pi;
Dt= 0.1;

if scenario == 1
    % trajetoria circular
    u = 0; r = -0.02;
    Dt= 0.1; t = 0:Dt:1.9*pi/-r;
    yaw = zeros(size(t))+pi/2;% +r*t;
    vx = u*cos(yaw);
    vy = u*sin(yaw);
    P0 = [20,-20];
    Px = P0(1) + cumsum(vx)*Dt; 
    Py = P0(2) + cumsum(vy)*Dt;

    Pd = [0,0,0];   % Dock [x,y,yaw]
    r = sqrt( (Px-Pd(1)).^2 + (Py -Pd(2)).^2 );
    b1 = pi/2-yaw + atan2(Px-Pd(1),Pd(2)-Py);
    b2 = atan2(Py-Pd(2),Px-Pd(1)) - Pd(3);
    
    r1_m = r + range_noise*randn(size(r));
    r2_m = r + range_noise*randn(size(r));
    b1_m = b1 + bearing_noise*randn(size(b1));
    b2_m = b2 + bearing_noise*randn(size(b1));

elseif scenario == 2  
    % trajetoria sinusoidal
    P_d = [0,0]; v_c = [0.2,0];
    t = 0:Dt:300;
    vx = zeros(size(t))+0.1+v_c(1); 
    vy = 2*sin(0.05*t)+0.1+v_c(2);
    P0 = [-100,-100];
    Px = P0(1) + cumsum(vx)*Dt;
    Py = P0(2) + cumsum(vy)*Dt;

    Px_m = Px + position_noise*randn(size(Px));
    Py_m = Py + position_noise*randn(size(Py));
    Pm = [Px;Py];


elseif scenario == 3 
    % trajetoria lawnmower
    % TODO
end
b1(1)*180/pi
b2(1)*180/pi
r(1)


%% simulate filter

% filter design
R = 1000*eye(2); 
Q = [1,0,0,0,0; ...
    0,1,0,0,0; ... 
    0,0,0.01,0,0;...
    0,0,0,0.1,0;...
    0,0,0,0,0.01];

% filter initialization
x = zeros(5,length(t));
x(:,1) = [Pm(1,1),Pm(2,1),0,0,0]';
e = zeros(2,length(t));
P = Q;
y = Pm;
for k=1:length(t)
    % predict step
    x_ = f(x(:,k),Dt);
    F = compute_F(x(:,k),Dt);
    P_ = F*P*F' + Q;
    
    % filter step
    H = compute_H(x_);
    K = (P*H')*inv(H*P_*H' + R);
    e(:,k) = y(:,k)-H*x_;
    x(:,k+1) = x_ + K*e(:,k);
    P = (eye(5) - K*H)*P_;
end

% plots
figure; plot(Py, Px, 'LineWidth', 2); hold on; plot(x(2,:), x(1,:), 'LineWidth', 2); grid on;

legend('Ground truth', 'Filter Prediction', 'Dock location', 'Location', 'best'); xlabel('East y [m]'); ylabel('North y [m]');


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

function y_p = h(x)
    y_p = [sqrt(x(1).^2 + x(2).^2); pi-x(4)];
end

function H = compute_H(x) % here x = x_hat (state prediction)
    H = zeros(2,5);
    H(1,1) = x(1)/sqrt( x(1)^2 + x(2)^2);
    H(1,2) = x(2)/sqrt( x(1)^2 + x(2)^2);
    H(2,3) = -1;
end