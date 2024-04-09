% preamble
clearvars;clc;close all;format longg;

% initial conditions and parameters
Pd = [100,120,170];
P0 = [0,50,0];
Dt=0.1;


% filter design
R_nav = 10*eye(4);
Q_nav = [1,  0.1, 0,  0   0;
        0.1, 1,   0,  0,  0;
        0,   0,   0.1,  0,  0;
        0,   0,   0,  1,  0;  
        0,   0,   0,  0,  0.01];
P_nav = Q_nav;
R_dock = 1*eye(3);
Q_dock = [  0.1,   0, 0;
            0, 0.1,   0;
            0,   0,   1];
P_dock = [  100,   10, 0;
            10, 100,   0;
            0,   0,   100];


%% simulation
% initializations
t = 0:Dt:200;
x = zeros(5,length(t));
x(:,1) = [P0(1),P0(2),0,P0(3),0]';
u = zeros(2, length(t));
y0 = measure(x(:,1),Pd);

x_nav = zeros(5,length(t));
x_nav(:,1) = [y0(1:4);0];
e_nav = zeros(4, length(t));

x_dock = zeros(3,length(t));
x_dock(:,1) = [0.1;0.1;0];
e_dock = zeros(3, length(t));
temp=zeros(3,length(t));

y = zeros(7,length(t));
for k=2:length(t)
    % =============== Read sensors ========================================
    y_ = measure(x(:,k-1),Pd);
    gyro = y_(end);
    y(:,k) = y_(1:end-1);

    % =============== Filter ==============================================
    % =-=-=-=-=-=-=-= Predict step =-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=
    % navigation filter --------------------------------
    x_nav_ = f_nav(x_nav(:,k-1),gyro,Dt);
    F = compute_F_nav(x_nav(:,k),Dt);
    %x_nav_ = F*x_nav(:,k);
    P_nav_ = F*P_nav*F' + Q_nav;
    % dock estimate filter ----------------------------
    P_dock_ = P_dock + Q_dock;

    % =-=-=-=-=-=-=-= Update step =-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=
    % navigation filter--------------------------------
    H =  [eye(4),[0,0,0,0]']; 
    K = (P_nav_*H')*inv(H*P_nav_*H' + R_nav);
    e_nav(:,k) = y(1:4,k) - H*x_nav_;
    x_nav(:,k) = x_nav_ + K*e_nav(:,k);
    P_nav = (eye(5) - K*H)*P_nav_;

    % dock estimate filter ----------------------------
    H = compute_H_dock(x_nav_, x_dock(:,k-1));
    K = (P_dock_*H')*inv(H*P_dock_*H' + R_dock);
    temp(:,k) = h_dock(x_nav_, x_dock(:,k-1));
    e_dock(:,k) = y(5:7,k)-h_dock(x_nav_, x_dock(:,k-1));
    x_dock(:,k) = x_dock(:,k-1) + K*e_dock(:,k);
    x_dock(3,k) = wrapTo180(x_dock(3,k));
    P_dock = (eye(3) - K*H)*P_dock_;

    % ============ Compute control ========================================
    u(:,k) = los_controller([x_nav(:,k); x_dock(:,k)]);

    % ============ Aply control to the plant ==============================
    x(:,k) = model(x(:,k-1), u(:,k), Dt);
    if sqrt((x(1,k)-Pd(1))^2 +(x(2,k)-Pd(2))^2) < 1
        break
    end
end
x = x(:,2:k);
x_nav = x_nav(:,2:k);
x_dock = x_dock(:,2:k);
y = y(:,2:k);
temp = temp(:,2:k);

% plots
figure; hold on; grid on;
%plot(y(2,:), y(1,:));
plot(x(2,:), x(1,:), 'LineWidth', 2); 
plot(x_nav(2,:), x_nav(1,:), 'LineWidth', 2); 
plot(Pd(2), Pd(1), '^', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
plot(tand(Pd(3))*linspace(Pd(1)-30, Pd(1), 10) + Pd(2) - tand(Pd(3))*Pd(1), linspace(Pd(1)-30, Pd(1), 10), 'b--', 'LineWidth', 2);
legend('Ground truth', 'Filter Prediction', 'Dock location', 'Location', 'best'); 
xlabel('East y [m]'); ylabel('North x [m]');
figure; hold on; grid on;
plot(x_dock(2,:),x_dock(1,:),'*')
figure; hold on; grid on;
plot(x_dock(3,:))
figure; hold on; grid on;
plot(y(5,:))
plot(temp(1,:))
figure; hold on; grid on;
plot(y(6,:))
plot(temp(2,:))
figure; hold on; grid on;
plot(y(7,:))
plot(temp(3,:))


