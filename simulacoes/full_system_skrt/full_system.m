% preamble
clearvars;clc;close all;format longg;

% initial conditions and parameters
Pd = [10,10,deg2rad(45)];
P0 = [0,0,0,0,0,0]';
Dt=0.1;
Vc = [0,0];


%% simulation
% initializations
t = 0:Dt:300;

% state for dynamics/kinematics modeling
x = zeros(6,length(t));
x(:,1) = P0;
u = zeros(3, length(t));
tau = zeros(2, length(t));
y = zeros(10,length(t));

controller = ua_controller(Dt);
yaw_pid = yaw_PID_controller();
surge_pid = surge_PID_controller();
sway_pid = sway_PID_controller();

y0 = measure(x(:,1),Pd);
X0 = [y0(1),y0(2),0,0,y0(3), y0(4)*cos(y0(5)+y0(3)), y0(4)*sin(y0(5)+y0(3)), y0(5)+y0(3),1,1,1]';
ekf = EKF(X0,Dt);
x_hat = zeros(11,length(t));
dock_yaw_cov = zeros(1,length(t));
dock_pos_cov = zeros(4,length(t));

debug = zeros(6,length(t));
state = ones(size(t));

% u(1,:) = ones(size(t))*0.4;
% u(2,:) = ones(size(t))*0.2;
% u(3,:) = zeros(size(t));
% u(3,1:30/Dt) = deg2rad(45);
% u(3,30/Dt+1:60/Dt) = deg2rad(-45);
% u(3,60/Dt+1:90/Dt) = deg2rad(170);
% u(3,90/Dt+1:120/Dt) = deg2rad(0);

for k=2:length(t)
    % =============== Simulate sensors ====================================
    y(:,k) = measure(x(:,k-1),Pd);

    % =============== Filter ==============================================
    ekf = ekf.predict(y(8:10,k));
    ekf = ekf.update(y(1:7,k));
    x_hat(:,k) = ekf.X;
    % debug(:,k) = ekf.debug;
    dock_yaw_cov(:,k) = ekf.P4;
    dock_pos_cov(:,k) = ekf.P3(:);

    % ============ Compute control ========================================
    % outer-loops
    controller.compute(x_hat(:,k), y(:,k));
    %debug(:,k)= controller.debug;
    u(:,k) = controller.output;
    state(k) = controller.state; 

    % inner-loops
    
    surge_pid.compute(y(9,k), u(1,k));
    sway_pid.compute(y(10,k), u(2,k));
    if controller.state == 2
        % use relative localization
        yaw_pid.compute(x_hat(11,k-1),u(3,k), y(8,k));
    else
        % use inertial localization
        yaw_pid.compute(x_hat(5,k-1), u(3,k), y(8,k));
    end
    debug(:,k) = yaw_pid.debug;
    tau(1,k) = surge_pid.output;
    tau(2,k) = sway_pid.output;
    tau(3,k) = yaw_pid.output;
    

    % ============ Aply control to the plant ==============================
    x(:,k) = model(x(:,k-1), tau(:,k), Dt,Vc);   

    if sqrt((x(1,k)-Pd(1))^2 +(x(2,k)-Pd(2))^2) < 0.2
        break    
    end
end

x = x(:,2:k);
x_hat = x_hat(:,2:k);
y = y(:,2:k);
u = u(:,2:k);
dock_yaw_cov = dock_yaw_cov(:,2:k);
dock_pos_cov = dock_pos_cov(:,2:k);
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

figure; hold on;grid on;
plot(t,zeros(size(t))+wrapToPi(Pd(3)));plot(t, x_hat(8,:));
fill([t, fliplr(t)], [x_hat(8,:)-2*dock_yaw_cov, fliplr(x_hat(8,:)+2*dock_yaw_cov)], 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Time');
ylabel('Dock Yaw [º]');
title('Docking Station orientation estimate');
hold off;


figure; hold on; grid on; axis equal;
xy = Rot(-Pd(3))*(x(1:2,:) - Pd(1:2)');
plot(xy(2,:), xy(1,:))
plot(x_hat(10,:), x_hat(9,:))
legend('Ground Truth', 'Filter Prediction', 'Location', 'best'); 
title('Range');


figure
plot(t,wrapTo180(45+rad2deg(x(3,:)-Pd(3))))
hold on
plot(t,45+rad2deg(u(3,:)))
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


