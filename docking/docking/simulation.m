% preamble
clearvars;clc;close all;format longg; addpath("controllers/");addpath("ploting/");

% Flag to turn estimator on or off
estimator_off = false;

% initial conditions and parameters
Pd = [0;0;2;0];                         % Dock Pose
P0 = [10;5;0;deg2rad(90);0;0;0;0];      % initial state:(x,y,z,yaw,u,v,w,r)
Dt=0.1;                                 % Timestep
Vc = [0;0;0];                           % Sea current velocity

% Controller Object 
controller = docking_controller();

%   Kalman Filter Object
y0 = measure(P0,Pd,Vc);
usbl_measurements = extract_observations(y0(5:7), y0(8:10));
kf = kalman_filter(usbl_measurements(1:2), usbl_measurements(3), usbl_measurements(4), y0(4),Dt);

% Initializations
t = 0:Dt:200;               % Simulation time
x = zeros(8,length(t));     % state for dynamics/kinematics modeling
x(:,1) = P0;                % Inital condition
tau = zeros(4, length(t));  % Control signal
y = zeros(14,length(t));    % Measurements vector
x_hat = zeros(4,length(t)); % Estimated state
usbl_measurements = nan(4,length(t)); % Estimated state

ref = zeros(6,length(t));   % Estimated state
S = zeros(4,length(t));     % S variable from SMC controllers
e = zeros(4,length(t));     % S variable from SMC controllers
debug = zeros(4,length(t)); % handy varaible for debuging
state = zeros(size(t));     % controller state
usbl_time_cnt = 0;

%% Simulation loop
for k=2:length(t)
    % =============== Simulate sensors ====================================
    y(:,k) = measure(x(:,k-1),Pd,Vc);

    % =============== Filter ==============================================
    usbl_time_cnt=usbl_time_cnt+1;
    if usbl_time_cnt >= 40
        if rand < 0.99
            %disp("Got fix wow miracle ")
            usbl_measurements(:,k) = extract_observations(y(5:7,k), y(8:10,k));
            kf = kf.update_usbl(usbl_measurements(1:2,k), usbl_measurements(3,k), usbl_measurements(4,k) );
        end
        usbl_time_cnt=0;
    end
    kf = kf.update_ahrs(y(4,k));
    kf = kf.predict(y(11:12,k), y(13,k), y(14,k));
    x_hat(:,k) = kf.X;
    x_true = x(1:4,k-1)-[0;0;2;0];
    
    if estimator_off
        x_hat(:,k) = x(1:4,k-1)-[0;0;2;0];
    end

    % ============ Compute control ========================================    
    controller = controller.compute(x_hat(:,k), y(11:13,k), y(14,k));
    tau(:,k) = controller.tau;
    state(:,k) = controller.state;
    S(:,k) = controller.s;
    Phis(:,k) = controller.Phis;
    e(:,k) = x_true - controller.ref(1:4);%controller.e;
    ref(:,k) = controller.ref;
   
    %TODO: thrust allocation

    % ============ Apply control to the plant ==============================
    x(:,k) = model(x(:,k-1), tau(:,k), Dt,Vc);   
        
    % stop if docking complete
    if sqrt((x(1,k)-Pd(1))^2 +(x(2,k)-Pd(2))^2) < 1 && x_hat(1,k) < 0 
        break    
    end
end

% Trim variables to avoid invalid indexes
x = x(:,2:k);
x_hat = x_hat(:,2:k);
y = y(:,2:k);
t = t(:,2:k);
tau = tau(:,2:k);
S = S(:,2:k);
Phis = Phis(:,2:k);
e = e(:,2:k);
usbl_measurements = usbl_measurements(:,2:k);
debug = debug(:,2:k);
ref = ref(:,2:k);
state = state(:,2:k);
S_dot = [0,diff(S(1,:))./diff(t);
            0,diff(S(2,:))./diff(t);
            0,diff(S(3,:))./diff(t);
            0,diff(S(4,:))./diff(t)];
e_dot = [diff(e(1,:))./diff(t);
            diff(e(2,:))./diff(t);
            diff(e(3,:))./diff(t);
            diff(e(4,:))./diff(t)];


%% PLOTS 
close all;

% Tracking Errors
fig =figure();
set(fig, 'Position', [100, 100, 1000, 1000])
subplot(4,1,1);hold on; grid on;
plot(t, ref(1,:),"k",LineWidth=2);
plot(t, x(5,:),"r", LineWidth=2);
title("Surge (PID)", 'Interpreter', 'latex','FontSize', 12);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("Velocity [m/s]", 'Interpreter', 'latex','FontSize', 12)
subplot(4,1,2);hold on; grid on;
plot(t, ref(2,:),"k",LineWidth=2);
plot(t, x(2,:),"r",LineWidth=2);
title("Cross-track ", 'Interpreter', 'latex','FontSize', 12);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$Position$ [m]", 'Interpreter', 'latex','FontSize', 12)

subplot(4,1,3);hold on; grid on;
plot(t, ref(3,:),"k",LineWidth=2);
plot(t, x(3,:)-2,"r",LineWidth=2);
title("Z-Alignment ", 'Interpreter', 'latex','FontSize', 12);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$Position$ [m]", 'Interpreter', 'latex','FontSize', 12)

subplot(4,1,4);hold on; grid on;
plot(t, wrapTo2Pi(ref(4,:)),"k",LineWidth=2);
%plot(t, wrapTo2Pi(ref(5,:)),"b",LineWidth=2);
plot(t, wrapTo2Pi(x(4,:)),"r",LineWidth=2);
title("Heading Tracking ", 'Interpreter', 'latex','FontSize', 12);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$Angle$ [rad]", 'Interpreter', 'latex','FontSize', 12)

% Estimator performance 
fig =figure();
set(fig, 'Position', [100, 100, 1000, 1000])
subplot(4,1,1);hold on; grid on;
plot(t, x(1,:),LineWidth=2);
plot(t, x_hat(1,:),LineWidth=2);
plot(t, usbl_measurements(1,:),"ko");
title("$\hat{x}_{D}$", 'Interpreter', 'latex','FontSize', 12);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$Position$ [m]", 'Interpreter', 'latex','FontSize', 12)
subplot(4,1,2);hold on; grid on;
plot(t, x(2,:),LineWidth=2);
plot(t, x_hat(2,:),LineWidth=2);
plot(t, usbl_measurements(2,:),"ko");
title("$\hat{y}_{D}$", 'Interpreter', 'latex','FontSize', 12);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$Position$ [m]", 'Interpreter', 'latex','FontSize', 12)
subplot(4,1,3);hold on; grid on;
plot(t, x(3,:)-2,LineWidth=2);
plot(t, x_hat(3,:),LineWidth=2);
plot(t, usbl_measurements(3,:),"ko");
title("$\hat{z}_{D}$", 'Interpreter', 'latex','FontSize', 12);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$Position$ [m]", 'Interpreter', 'latex','FontSize', 12)
subplot(4,1,4);hold on; grid on;
plot(t, wrapTo2Pi(x(4,:)),LineWidth=2);
plot(t, wrapTo2Pi(x_hat(4,:)),LineWidth=2);
plot(t, wrapTo2Pi(usbl_measurements(4,:)),"ko");
title("$\hat{\psi}_{D}$", 'Interpreter', 'latex','FontSize', 12);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$Angle$ [rad]", 'Interpreter', 'latex','FontSize', 12)

% actuation
fig = figure();
set(fig, 'Position', [100, 100, 1000, 1000])
title("Control Signals", 'Interpreter', 'latex','FontSize', 12);
subplot(4,1,1);hold on; grid on;
plot(t, tau(1,:),"k",LineWidth=2);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$\tau_u$ [N]", 'Interpreter', 'latex','FontSize', 12)
subplot(4,1,2);hold on; grid on;
plot(t, tau(2,:),"k",LineWidth=2);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$\tau_v$ [N]", 'Interpreter', 'latex','FontSize', 12)
subplot(4,1,3);hold on; grid on;
plot(t, tau(3,:),"k",LineWidth=2);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$\tau_w$ [N]", 'Interpreter', 'latex','FontSize', 12)
subplot(4,1,4);hold on; grid on;
plot(t, tau(4,:),"k",LineWidth=2);
xlabel("$t$ [s]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$\tau_{\psi}$ [N]", 'Interpreter', 'latex','FontSize', 12)


% Sliding surfaces
fig = figure();
set(fig, 'Position', [100, 100, 1000, 1000])
subplot(2,2,1);hold on; grid on;
plot( linspace(min(e(1,2:end)), max(e(1,2:end))), -linspace(min(e(1,2:end)), max(e(1,2:end))), LineWidth=2.5);
plot(e(1,2:end), e_dot(1,:), LineWidth=2);
title("Surge (PID)", 'Interpreter', 'latex','FontSize', 12);
legend({"trajectory", "$s = 0$"}, 'Interpreter', 'latex', 'Location', 'Best')
xlabel("$e_u$ [m]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$\dot{e}_u$ [m/s]", 'Interpreter', 'latex','FontSize', 12)

subplot(2,2,2);hold on; grid on;
plot( linspace(min(e(2,2:end)), max(e(2,2:end))), -linspace(min(e(2,2:end)), max(e(2,2:end))), LineWidth=2.5 )
plot(e(2,2:end), e_dot(2,:), LineWidth=2);
title("Cross-Track ", 'Interpreter', 'latex','FontSize', 12);
legend({"trajectory", "$s = 0$"}, 'Interpreter', 'latex', 'Location', 'Best')
xlabel("$e_y [m]$", 'Interpreter', 'latex','FontSize', 12)
ylabel("$\dot{e}_y$ [m/s]", 'Interpreter', 'latex','FontSize', 12)

subplot(2,2,3);hold on; grid on;
plot( linspace(min(e(3,2:end)), max(e(3,2:end))), -linspace(min(e(3,2:end)), max(e(3,2:end))), LineWidth=2.5 )
plot(e(3,2:end), e_dot(3,:), LineWidth=2);
title("Z-alignment ", 'Interpreter', 'latex','FontSize', 12);
legend({"trajectory", "$s = 0$"}, 'Interpreter', 'latex', 'Location', 'Best')
xlabel("$e_z$ [m]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$\dot{e}_z$ [m/s]", 'Interpreter', 'latex','FontSize', 12)

subplot(2,2,4);hold on; grid on;
title("Heading ", 'Interpreter', 'latex');
plot( linspace(min(e(4,2:end)), max(e(4,2:end))), -linspace(min(e(4,2:end)), max(e(4,2:end))), LineWidth=2.5 )
plot(e(4,2:end), e_dot(4,:), LineWidth=2);
legend({"trajectory", "$s = 0$"}, 'Interpreter', 'latex', 'Location', 'Best')
xlabel("$e_{\psi}$ [rad]", 'Interpreter', 'latex','FontSize', 12)
ylabel("$\dot{e}_{\psi}$ [rad/s]", 'Interpreter', 'latex','FontSize', 12)

% s and Boundary layer
fig = figure();
set(fig, 'Position', [100, 100, 1000, 1000])
subplot(2,2,1);hold on; grid on;
plot(t, Phis(1,:), "k", LineWidth=2);
plot(t, -Phis(1,:), "k", LineWidth=2);
plot(t, S(1,:), "b", LineWidth=3);
title("Surge (PID)", 'Interpreter', 'latex','FontSize', 12);
%legend({"trajectory", "$s = 0$"}, 'Interpreter', 'latex', 'Location', 'Best')
%xlabel("$e_u$ [m]", 'Interpreter', 'latex','FontSize', 12)
%ylabel("$\dot{e}_u$ [m/s]", 'Interpreter', 'latex','FontSize', 12)

subplot(2,2,2);hold on; grid on;
plot(t, Phis(2,:), "k", LineWidth=2);
plot(t, -Phis(2,:), "k", LineWidth=2);
plot(t, S(2,:), "b", LineWidth=3);
title("cross-track (y)", 'Interpreter', 'latex','FontSize', 12);
%legend({"trajectory", "$s = 0$"}, 'Interpreter', 'latex', 'Location', 'Best')
%xlabel("$e_u$ [m]", 'Interpreter', 'latex','FontSize', 12)
%ylabel("$\dot{e}_u$ [m/s]", 'Interpreter', 'latex','FontSize', 12)

subplot(2,2,3);hold on; grid on;
plot(t, Phis(3,:), "k", LineWidth=2);
plot(t, -Phis(3,:), "k", LineWidth=2);
plot(t, S(3,:), "b", LineWidth=3);
title("z", 'Interpreter', 'latex','FontSize', 12);
%legend({"trajectory", "$s = 0$"}, 'Interpreter', 'latex', 'Location', 'Best')
%xlabel("$e_u$ [m]", 'Interpreter', 'latex','FontSize', 12)
%ylabel("$\dot{e}_u$ [m/s]", 'Interpreter', 'latex','FontSize', 12)

subplot(2,2,4);hold on; grid on;
plot(t, Phis(4,:), "k", LineWidth=2);
plot(t, -Phis(4,:), "k", LineWidth=2);
plot(t, S(4,:), "b", LineWidth=3);
title("heading", 'Interpreter', 'latex','FontSize', 12);
%legend({"trajectory", "$s = 0$"}, 'Interpreter', 'latex', 'Location', 'Best')
%xlabel("$e_u$ [m]", 'Interpreter', 'latex','FontSize', 12)
%ylabel("$\dot{e}_u$ [m/s]", 'Interpreter', 'latex','FontSize', 12)











% fig = figure(); hold on; grid on;
% plot(x(2,:), x(1,:));
% figure();hold on; grid on;
% plot(t,S(1,:));plot(t,S(2,:));plot(t,S(3,:));plot(t,S(4,:));
% figure();hold on; grid on;
% plot(t,tau(1,:));plot(t,tau(2,:));plot(t,tau(3,:));plot(t,tau(4,:));
% figure();hold on; grid on;
% plot(t,x(4,:));
% figure();hold on; grid on;
% plot(t,x(5,:));plot(t,x(6,:));plot(t,x(8,:));

% XY plot
% fig = figure(); hold on; grid on;
% % docking station for legend
% plot(-999,-999, 's', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black', 'MarkerSize', 10);
% % homing point
% plot(0,20, 'x', 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red', 'MarkerSize', 10, 'LineWidth',2);
% % path 
% plot([0,0], [0,20], '--', 'Color',[0.5,0.5,0.5], LineWidth=2);
% % real trajectory
% xy = Rot(-Pd(3))*(x(1:2,:) - Pd(1:2)'); plot(xy(2,:), xy(1,:), Color=[0 0.4470 0.7410], LineWidth=2)
% % filter estimate
% plot(x_hat(2,2:end), x_hat(1,2:end), Color=[0.8500 0.3250 0.0980], LineWidth=2)
% % draw DS
% plotFilledTriangle([0,0], .5, 0, 'black');
% % draw auv at given positions
% nr_samples = 8;
% samples_time = [t(1:int32(length(t)/(nr_samples-1)):end-40)]; 
% a = find(ismember(t, samples_time) == 1);
% samples_mvector = [xy(2,a); xy(1,a); x(3,a)-Pd(3)].';        
% for i = 1:length(samples_mvector)
%     GTF_Simulink_PlotAUV([samples_mvector(i,1),samples_mvector(i,2),0], [0,0,-samples_mvector(i,3)*180/pi-90], 0.1, 0, [0.9290 0.6940 0.1250],1);
% end
% %labels and shit
% xlabel('$y^{\mathcal{D}}$ [m]', 'Interpreter','latex'); ylabel('$x^{\mathcal{D}}$ [m]','Interpreter','latex');
% legend('Docking Station', 'Homing Point','Reference Trajectory','Real Trajectory','Filter Prediction', 'Location', 'best'); 
% 
% xlim([-1 20.5])
% ylim([-1 20.5])
% %axis equal
% set(gcf, 'Position', [100, 100, 600, 600]); 
% %print(fig, '../../imagens_pic/xy.png', '-dpng', '-r300'); 


% % heading plot
% fig = figure();hold on; grid on;
% plot(t,wrapTo360(rad2deg(x(4,:)-Pd(3))), Color=[0 0.4470 0.7410], LineWidth=2) 
% plot(t,wrapTo360(rad2deg(x_hat(5,:))), Color=[0.8500 0.3250 0.0980], LineWidth=2)
% legend('Real Heading','Estimated Heading', 'Location', 'best'); 
% xlim([min(t), max(t) ]);
% xlabel('Time [s]', 'Interpreter','latex'); ylabel('Heading [º]');
% set(gcf, 'Position', [100, 100, 600, 200]); 
% %print(fig, '../../imagens_pic/yaw.png', '-dpng', '-r300'); 
% 
% % veolicty plot
% fig = figure();hold on; grid on;
% plot(t, y(10,:), Color=[0 0.4470 0.7410], LineWidth=2) 
% plot(t, y(9,:), Color=[0.8500 0.3250 0.0980], LineWidth=2)
% legend('Surge','Sway', 'Location', 'best'); 
% xlim([min(t), max(t) ]);
% xlabel('Time [s]', 'Interpreter','latex'); ylabel('Speed [m/s]');
% set(gcf, 'Position', [100, 100, 600, 200]); 
% %print(fig, '../../imagens_pic/uv.png', '-dpng', '-r300'); 
% 
% 
% % filter errors
% fig = figure();hold on; grid on;
% % position
% yyaxis left;
% error = sqrt((x_hat(1,2:end)-xy(1,2:end)).^2 + (x_hat(2,2:end)-xy(2,2:end)).^2 ) ;
% plot(t(2:end),error, Color=[0.1059 0.3686 0.1255], LineWidth=2) 
% ylabel('Position Error [m]','Interpreter','latex');
% set(gca, 'ycolor', [0.1059 0.3686 0.1255]); 
% % heading
% yyaxis right;
% error = wrapToPi(x_hat(5,3:end)-x(3,3:end)+Pd(3));
% plot(t(3:end),error, Color=[0.6350 0.0780 0.1840], LineWidth=2) 
% ylabel('Heading Error [º]');
% set(gca, 'ycolor', [0.6350 0.0780 0.1840]); 
% xlabel('Time [s]', 'Interpreter','latex');
% xlim([min(t), max(t) ])
% set(gcf, 'Position', [100, 100, 600, 150]);
% %print(fig, '../../imagens_pic/filter_error.png', '-dpng', '-r300'); 
% 
% 
% % control errors ig
% fig = figure();hold on; grid on;
% a = find(state==2);
% % position
% yyaxis left;
% 
% error = xy(2,a);
% plot(t(a),error, Color=[0.1059 0.3686 0.1255], LineWidth=2)
% ylabel('Cross-track Error [m]','Interpreter','latex');
% set(gca, 'ycolor', [0.1059 0.3686 0.1255]); 
% % heading
% yyaxis right;
% error = 180/pi*wrapToPi(pi-x(3,a)+Pd(3));
% plot(t(a),error, Color=[0.6350 0.0780 0.1840], LineWidth=2) 
% ylim([min(error), max(error)+2])
% ylabel('Heading Error [º]');
% set(gca, 'ycolor', [0.6350 0.0780 0.1840]);
% xlim([min(t(a)), max(t(a)) ])
% xlabel('Time [s]', 'Interpreter','latex');
% set(gcf, 'Position', [100, 100, 600, 150]); 
% %print(fig, '../../imagens_pic/docking_error.png', '-dpng', '-r300'); 
% 
% 
