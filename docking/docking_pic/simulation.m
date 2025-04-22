% preamble
clearvars;clc;close all;format longg;

% initial conditions and parameters
Pd = [10,10,deg2rad(90)];
P0 = [-5,20,deg2rad(0),0,0,0]';
Dt=0.1;
Vc = [0.2;0];


% simulation
% initializations
t = 0:Dt:500;

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

kf = kalman_filter(measure(x(:,1),Pd,Vc),Dt);
x_hat = zeros(5,length(t));

debug = zeros(1,length(t));
state = ones(size(t));

% u(1,:) = ones(size(t))*0;
% u(2,:) = ones(size(t))*0.5;
% u(3,:) = zeros(size(t));
% u(2,1:30/Dt) = 0.4;
% u(2,30/Dt+1:60/Dt) = 0.0;
% u(2,60/Dt+1:90/Dt) = 0.2;
% u(2,90/Dt+1:120/Dt) = 0.0;

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
    u(:,k) = controller.output;
    state(k) = controller.state; 

    % inner-loops
    surge_pid.compute(y(9,k), u(1,k));
    sway_pid.compute(y(10,k), u(2,k));
    yaw_pid.compute(x_hat(5,k), u(3,k), y(8,k));

    tau(1,k) = surge_pid.output;
    tau(2,k) = sway_pid.output;
    tau(3,k) = yaw_pid.output;
    
    % ============ Aply control to the plant ==============================
    x(:,k) = model(x(:,k-1), tau(:,k), Dt,Vc);   
        
    % stop if docking complete
    if sqrt((x(1,k)-Pd(1))^2 +(x(2,k)-Pd(2))^2) < 1 && x_hat(1,k) < 0 
        break    
    end
end

x = x(:,1:k);
x_hat = x_hat(:,1:k);
y = y(:,1:k);
u = u(:,1:k);
t = t(:,1:k);

%% PLOTS 
% XY plot
fig = figure(); hold on; grid on;
% docking station for legend
plot(-999,-999, 's', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black', 'MarkerSize', 10);
% homing point
plot(0,20, 'x', 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red', 'MarkerSize', 10, 'LineWidth',2);
% path 
plot([0,0], [0,20], '--', 'Color',[0.5,0.5,0.5], LineWidth=2);
% real trajectory
xy = Rot(-Pd(3))*(x(1:2,:) - Pd(1:2)'); plot(xy(2,:), xy(1,:), Color=[0 0.4470 0.7410], LineWidth=2)
% filter estimate
plot(x_hat(2,2:end), x_hat(1,2:end), Color=[0.8500 0.3250 0.0980], LineWidth=2)
% draw DS
plotFilledTriangle([0,0], .5, 0, 'black');
% draw auv at given positions
nr_samples = 8;
samples_time = [t(1:int32(length(t)/(nr_samples-1)):end-40)]; 
a = find(ismember(t, samples_time) == 1);
samples_mvector = [xy(2,a); xy(1,a); x(3,a)-Pd(3)].';        
for i = 1:length(samples_mvector)
    GTF_Simulink_PlotAUV([samples_mvector(i,1),samples_mvector(i,2),0], [0,0,-samples_mvector(i,3)*180/pi-90], 0.1, 0, [0.9290 0.6940 0.1250],1);
end
%labels and shit
xlabel('$y^{\mathcal{D}}$ [m]', 'Interpreter','latex'); ylabel('$x^{\mathcal{D}}$ [m]','Interpreter','latex');
legend('Docking Station', 'Homing Point','Reference Trajectory','Real Trajectory','Filter Prediction', 'Location', 'best'); 

xlim([-1 20.5])
ylim([-1 20.5])
%axis equal
set(gcf, 'Position', [100, 100, 600, 600]); 
%print(fig, '../../imagens_pic/xy.png', '-dpng', '-r300'); 


% heading plot
fig = figure();hold on; grid on;
plot(t,wrapTo360(rad2deg(x(3,:)-Pd(3))), Color=[0 0.4470 0.7410], LineWidth=2) 
plot(t,wrapTo360(rad2deg(x_hat(5,:))), Color=[0.8500 0.3250 0.0980], LineWidth=2)
legend('Real Heading','Estimated Heading', 'Location', 'best'); 
xlim([min(t), max(t) ]);
xlabel('Time [s]', 'Interpreter','latex'); ylabel('Heading [º]');
set(gcf, 'Position', [100, 100, 600, 200]); 
%print(fig, '../../imagens_pic/yaw.png', '-dpng', '-r300'); 

% veolicty plot
fig = figure();hold on; grid on;
plot(t, y(10,:), Color=[0 0.4470 0.7410], LineWidth=2) 
plot(t, y(9,:), Color=[0.8500 0.3250 0.0980], LineWidth=2)
legend('Surge','Sway', 'Location', 'best'); 
xlim([min(t), max(t) ]);
xlabel('Time [s]', 'Interpreter','latex'); ylabel('Speed [m/s]');
set(gcf, 'Position', [100, 100, 600, 200]); 
%print(fig, '../../imagens_pic/uv.png', '-dpng', '-r300'); 


% filter errors
fig = figure();hold on; grid on;
% position
yyaxis left;
error = sqrt((x_hat(1,2:end)-xy(1,2:end)).^2 + (x_hat(2,2:end)-xy(2,2:end)).^2 ) ;
plot(t(2:end),error, Color=[0.1059 0.3686 0.1255], LineWidth=2) 
ylabel('Position Error [m]','Interpreter','latex');
set(gca, 'ycolor', [0.1059 0.3686 0.1255]); 
% heading
yyaxis right;
error = wrapToPi(x_hat(5,3:end)-x(3,3:end)+Pd(3));
plot(t(3:end),error, Color=[0.6350 0.0780 0.1840], LineWidth=2) 
ylabel('Heading Error [º]');
set(gca, 'ycolor', [0.6350 0.0780 0.1840]); 
xlabel('Time [s]', 'Interpreter','latex');
xlim([min(t), max(t) ])
set(gcf, 'Position', [100, 100, 600, 150]);
%print(fig, '../../imagens_pic/filter_error.png', '-dpng', '-r300'); 


% control errors ig
fig = figure();hold on; grid on;
a = find(state==2);
% position
yyaxis left;

error = xy(2,a);
plot(t(a),error, Color=[0.1059 0.3686 0.1255], LineWidth=2)
ylabel('Cross-track Error [m]','Interpreter','latex');
set(gca, 'ycolor', [0.1059 0.3686 0.1255]); 
% heading
yyaxis right;
error = 180/pi*wrapToPi(pi-x(3,a)+Pd(3));
plot(t(a),error, Color=[0.6350 0.0780 0.1840], LineWidth=2) 
ylim([min(error), max(error)+2])
ylabel('Heading Error [º]');
set(gca, 'ycolor', [0.6350 0.0780 0.1840]);
xlim([min(t(a)), max(t(a)) ])
xlabel('Time [s]', 'Interpreter','latex');
set(gcf, 'Position', [100, 100, 600, 150]); 
%print(fig, '../../imagens_pic/docking_error.png', '-dpng', '-r300'); 



% % cross track error 
% figure;hold on; grid on;
% error = xy(2,2:end);
% plot(t(2:end),error, Color=[0 0 0], LineWidth=2) 
% xlabel('Time [s]', 'Interpreter','latex'); ylabel('cross-track error [º]');
% set(gcf, 'Position', [100, 100, 600, 150]); 
% 
% % heading error
% figure;hold on; grid on;
% error = 180/pi*wrapToPi(pi-x(3,2:end)+Pd(3));
% plot(t(2:end),error, Color=[0 0 0], LineWidth=2) 
% xlabel('Time [s]', 'Interpreter','latex'); ylabel('Heading error[º]');
% set(gcf, 'Position', [100, 100, 600, 150]); 


% Trajectory
% figure; hold on; grid on;
% plot(x(2,:), x(1,:), 'LineWidth', 2); 
% plot(x_hat(2,:), x_hat(1,:), 'LineWidth', 1); 
% plot(Pd(2)-0.5, Pd(1), 's', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black', 'MarkerSize', 10);
% plot(Pd(2)+20*sin(Pd(3)),Pd(1)+20*cos(Pd(3)), 'x', 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red', 'MarkerSize', 10, 'LineWidth',5);
% plot([Pd(2),Pd(2)+20*sin(Pd(3))], [Pd(1),Pd(1)+20*cos(Pd(3))], '--', 'Color',[0.5,0.5,0.5],  'LineWidth', 2);
% 
% 
% nr_samples = 7;
% samples_time = [t(1:length(t)/(nr_samples-1):end) t(end-40)];
% a = find(ismember(t, samples_time) == 1);
% samples_mvector = [x(2,a); x(1,a); x(3,a)].';        
% for i = 1:length(samples_mvector)
%     GTF_Simulink_PlotAUV([samples_mvector(i,1),samples_mvector(i,2),0], [0,0,-samples_mvector(i,3)*180/pi-90], 0.1, 0, [0.9290 0.6940 0.1250],1);
% end
% plotFilledTriangle(Pd(1:2), .5, Pd(3)*180/pi, 'black');
% legend('Ground truth', 'Dock location','Homing point','Path', 'Location', 'best'); 
% xlabel('$y^{\mathcal{D}}$ [m]', 'Interpreter','latex'); ylabel('$x^{\mathcal{D}}$ [m]','Interpreter','latex');
%axis equal
% xlim([5, 35])
% ylim([-2, 12])

% % Left y-axis
% yyaxis left;
% plot(x, y1, 'b-', 'LineWidth', 2); % Plotting first dataset in blue
% ylabel('Left Scale');
% 
% % Right y-axis
% yyaxis right;
% plot(x, y2, 'r-', 'LineWidth', 2); % Plotting second dataset in red
% ylabel('Right Scale');

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
% figure; hold on; grid on;
% plot(state(2:k))
% title('Controller state');
%
% figure; hold on; grid on;
% plot(u(2,2:k))
% title('Yaw desired');


