% preamble
clearvars;clc;close all;format longg;

% initial conditions and parameters
Pd = [50,50,45];
P0 = [0,0,0,0]';
Dt=0.1;
Vc = [0,0];


%% simulation
% initializations
t = 0:Dt:500;

% state for dynamics/kinematics modeling
x = zeros(4,length(t));
x(:,1) = P0;
u = zeros(2, length(t));
y = zeros(9,length(t));

controller = LOS_controller();
controller.output

y0 = measure(x(:,1),Pd);
X0 = [y0(1),y0(2),0,0,y0(3), 10, 10, 0,1,1,1]';
ekf = EKF(X0,Dt);
x_hat = zeros(11,length(t));
debug = zeros(2,length(t));
dock_yaw_cov = zeros(1,length(t));
dock_pos_cov = zeros(4,length(t));

stage = ones(size(t));
for k=2:length(t)
    % =============== Read sensors ========================================
    y(:,k) = measure(x(:,k-1),Pd);

    % =============== Filter ==============================================
    ekf = ekf.predict(y(8,k));
    ekf = ekf.update(y(1:7,k));
    x_hat(:,k) = ekf.X;
    debug(:,k) = ekf.debug;
    dock_yaw_cov(:,k) = ekf.P4;
    dock_pos_cov(:,k) = ekf.P3(:);

    % ============ Compute control ========================================
    controller.compute(x_hat(:,k), y(:,k));
    u(:,k) = controller.output;

    % ============ Aply control to the plant ==============================
    x(:,k) = model(x(:,k-1), u(:,k), Dt,Vc);


    % make cool movie plot

    % clf;
    % figure; hold on; grid on;
    % plot(x(2,:), x(1,:), 'LineWidth', 2); 
    % plot(x_hat(2,:), x_hat(1,:), 'LineWidth', 1); 
    % plot(Pd(2), Pd(1), '^', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
    % plot([Pd(2),Pd(2)+50*sind(Pd(3))], [Pd(1),Pd(1)+50*cosd(Pd(3))], 'b--', 'LineWidth', 2);
    % legend('Ground truth', 'Filter Prediction', 'Dock location', 'Location', 'best'); 
    % xlabel('East y [m]'); ylabel('North x [m]');
    % 
    % Z = mvnpdf([x(:) y(:)], mu, Sigma);
    % Z = reshape(Z, size(x));
    % contour(x, y, Z);

    if sqrt((x(1,k)-Pd(1))^2 +(x(2,k)-Pd(2))^2) < 1
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
figure; hold on; grid on;
plot(x(2,:), x(1,:), 'LineWidth', 2); 
plot(x_hat(2,:), x_hat(1,:), 'LineWidth', 1); 
plot(Pd(2), Pd(1), '^', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
plot([Pd(2),Pd(2)+50*sind(Pd(3))], [Pd(1),Pd(1)+50*cosd(Pd(3))], 'b--', 'LineWidth', 2);
legend('Ground truth', 'Filter Prediction', 'Dock location','Path', 'Location', 'best'); 
xlabel('East y [m]'); ylabel('North x [m]');

% figure; hold on; grid on;
% plot(zeros(size(t))+wrapTo180(Pd(3)));plot(x_hat(8,:));
% title('Dock Yaw');
% legend('Real', 'Filter Prediction', 'Location', 'best'); 


% Plotting
figure; hold on;grid on;
plot(t,zeros(size(t))+wrapTo180(Pd(3)));plot(t, x_hat(8,:));
fill([t, fliplr(t)], [x_hat(8,:)-2*dock_yaw_cov, fliplr(x_hat(8,:)+2*dock_yaw_cov)], 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Time');
ylabel('Dock Yaw [º]');
title('Docking Station orientation estimate');
hold off;


% figure; hold on; grid on;
% plot(y(5,:))
% plot(temp(1,:))
% legend('Measured', 'Filter Prediction', 'Location', 'best'); 
% title('Range');
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
plot(stage(2:k))
title('Controller state');
% 
% figure; hold on; grid on;
% plot(u(2,2:k))
% title('Yaw desired');


