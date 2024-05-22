% preamble
clearvars;clc;close all;format longg;

% initial conditions and parameters
Pd = [10,10,deg2rad(90)];
P0 = [6,30,deg2rad(-90),0,0,0]';
Dt=0.1;
Vc = [0.2;0];


%% simulation
% initializations
t = 0:Dt:500;

% state for dynamics/kinematics modeling
x = zeros(6,length(t));
x(:,1) = P0;
u = zeros(3, length(t));
tau = zeros(2, length(t));
y = zeros(10,length(t));

controller = fa_controller(Dt);
yaw_pid = yaw_PID_controller();
surge_pid = surge_PID_controller();
sway_pid = sway_PID_controller();

kf = kalman_filter(measure(x(:,1),Pd,Vc),Dt);
x_hat = zeros(5,length(t));

debug = zeros(2,length(t));
state = ones(size(t));

% u(1,:) = ones(size(t))*0.4;
% u(2,:) = ones(size(t))*0;
% u(3,:) = zeros(size(t));
% u(3,1:30/Dt) = deg2rad(90);
% u(3,30/Dt+1:60/Dt) = deg2rad(220);
% u(3,60/Dt+1:90/Dt) = deg2rad(45);
% u(3,90/Dt+1:120/Dt) = deg2rad(-45);

for k=2:length(t)
    % =============== Simulate sensors ====================================
    y(:,k) = measure(x(:,k-1),Pd,Vc);

    % =============== Filter ==============================================
    kf = kf.update(y(:,k));
    x_hat(:,k) = kf.X;
    %debug(:,k) = kf.debug;

    % ============ Compute control ========================================
    % outer-loops
    controller.compute(x_hat(:,k), y(:,k));
    debug(:,k)= controller.debug;
    u(:,k) = controller.output;
    state(k) = controller.state; 

    % inner-loops
    surge_pid.compute(y(9,k), u(1,k));
    sway_pid.compute(y(10,k), u(2,k));
    if controller.state == 0
        % use inertial localization
        yaw_pid.compute(y(5,k),u(3,k), y(8,k));
    else
        % use relative localization
        yaw_pid.compute(x_hat(5,k), u(3,k), y(8,k));
    end
    tau(1,k) = surge_pid.output;
    tau(2,k) = sway_pid.output;
    tau(3,k) = yaw_pid.output;
    

    % ============ Aply control to the plant ==============================
    x(:,k) = model(x(:,k-1), tau(:,k), Dt,Vc);   

    if sqrt((x(1,k)-Pd(1))^2 +(x(2,k)-Pd(2))^2) < 0.1
        break    
    end
end

x = x(:,2:k);
x_hat = x_hat(:,2:k);
y = y(:,2:k);
u = u(:,2:k);
t = t(:,2:k);

%% plots
% Trajectory
figure; hold on; grid on;
plot(x(2,:), x(1,:), 'LineWidth', 2); 
%plot(x_hat(2,:), x_hat(1,:), 'LineWidth', 1); 
plot(Pd(2), Pd(1), '^', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
plot(Pd(2)+20*sin(Pd(3)),Pd(1)+20*cos(Pd(3)), 'x', 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red', 'MarkerSize', 10);
plot([Pd(2),Pd(2)+20*sin(Pd(3))], [Pd(1),Pd(1)+20*cos(Pd(3))], 'b--', 'LineWidth', 2);
legend('Ground truth', 'Dock location','Homing point','Path', 'Location', 'best'); 
xlabel('East y [m]'); ylabel('North x [m]');
axis equal;



figure; hold on; grid on; axis equal;
xy = Rot(-Pd(3))*(x(1:2,:) - Pd(1:2)');
plot(xy(2,:), xy(1,:))
plot(x_hat(2,:), x_hat(1,:))
plot(0,0, '^', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
legend('Ground Truth', 'Filter Prediction','Docking Station', 'Location', 'best'); 
title('Range');


figure
plot(t,wrapTo360(rad2deg(x(3,:)-Pd(3))))
hold on
plot(t,wrapTo360(rad2deg(x_hat(5,:))))
plot(t, wrapTo360(rad2deg(u(3,:))))
% 
% figure; hold on; grid on;
% plot(y(6,:))
% plot(temp(2,:))
% legend('Measured', 'Filter Prediction', 'Location', 'best'); 
% title('AUV Bearing');
% 
% figure; hold on; grid on;
% plot(y(7,:))
% plot(temp(3,:))
% legend('Measured', 'Filter Prediction', 'Location', 'best'); 
% title('Dock Bearing');
% 
figure; hold on; grid on;
plot(state(2:k))
title('Controller state');
% 
% figure; hold on; grid on;
% plot(u(2,2:k))
% title('Yaw desired');


