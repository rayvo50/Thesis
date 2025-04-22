% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%   Author: Ravi Regalo <ravi.regalo@tecnico.ulisboa.pt>
%   
%   This contains a simulation of an AUV performing bottom following.
%   The simulation is done in a way that makes it easy to implement a
%   diferent controller or estimator. 
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ===  PREAMBLE ===
clearvars;clc;close all;format longg;

% initial conditions and parameters
output_folder_name = "outliers_filtered";
mkdir("imgs/"+output_folder_name);
movie = false;  % Set to true to generate a movie of the AUV
d_ref =2;       % Distance reference
x0 = [0,0,0,0]; % initial state 
Dt=0.1;         % Timestep for simulation and for controller/estimator
Vc = [0;0];     % Sea current velocity (not implemented)
 
alpha = 30*pi/180;  % Instalation angle of the 2nd echosounder

% gaussian noise profile generator (to add to terrain profile)
noise = gaussian_pseudo_random_noise();   

beta = 30*pi/180;   % terrain inclination [rads]
terrain = @(x) -tan(beta)*x+13;% + 0.1*noise.compute(x);
terrain_noiseless = @(x) -tan(beta)*x+13;

% % % For noisy terrain uncomment bellow
% terrain = @(x) -tan(beta)*x+10 + 0.1*noise.compute(x);
% terrain_noiseless = @(x) -tan(beta)*x+10;
% 
% % % For sinusoidal terrain uncomment beloow
% terrain = @(x) -2*sin(x/2)+10;
% terrain_noiseless = @(x) -2*sin(x/2)+10;

% intialize outlier rejector object
outlier_rejector = outlier_rejection(20,5.0,2); % window_size, threshold, y_DIM

% initializations
t = 0:Dt:100;

% state for dynamics/kinematics modeling
x = zeros(4,length(t));         % real state of the vehicle
x(:,1) = x0;    
tau = zeros(2, length(t));      % input forces
y = zeros(4,length(t));         % measurement
x_hat = zeros(4,length(t));     % state for the estimator
real_d = zeros(2,length(t));    % real D vector for comparison
debug = zeros(2,length(t));     % handy variable for debug
y_unfilt = zeros(2,length(t));  % y without outlier rejection 
V_ref = zeros(2, length(t));      % velocity references to the inner-loops

% Intialize inner-loop controllers
surge_pid = surge_PID_controller();
heave_pid = heave_PID_controller();

% Initial measurement and estiamtor initialization
y0 = measure(x(:,1), terrain, alpha);
qlkf = qLKF2(y0(3:4),Dt, alpha);


%% === RUN SIMULATION LOOP ===

if movie
    fig = figure(); hold on; grid on; axis equal; %axis([0,20,-20,0])
    plot(linspace(0,20), -terrain(linspace(0,20)), 'b-', 'LineWidth', 1.5);
    auv = plot(x0(1), -x0(2), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    ray1 = quiver(x0(1), x0(2), 0, 0, 0, 'r', 'LineWidth', 2);
    ray2 = quiver(x0(1), x0(2), 0, 0, 0, 'g', 'LineWidth', 2);
    D_est = quiver(x0(1), x0(2), 0, 0, 0, 'b', 'LineWidth', 2);
end
for k=2:length(t)
    % ============ Simulate sensors =======================================
    y(:,k) = measure(x(:,k-1),terrain, alpha);
    y_unfilt(:,k)=y(3:4,k);
    [outlier_rejector, y(3:4,k)] = outlier_rejector.compute(y(3:4,k));
    

    % ============ Estimator ==============================================
    qlkf = qlkf.compute(y(1:2,k), y(3:4,k));
    x_hat(1:2,k) = qlkf.xhat(1:2);
    x_hat(3:4,k) = qlkf.D_dot;
    real_d(:,k) = get_real_d(x(:,k-1), terrain_noiseless);

    
    % ============ Compute control ========================================
    [tau(:,k), V_ref(:,k)] = controller(x_hat(:,k), d_ref, 0.2, x(:,k-1), surge_pid, heave_pid );


    % ============ Aply control to the plant ==============================
    x(:,k) = model(x(:,k-1), tau(:,k), Dt);   


    % ============ Make movie Plot ========================================
    if movie
        set(auv, 'XData', x(1,k), 'YData', -x(2,k));
        set(ray1, 'XData', x(1,k), 'YData', -x(2,k), 'UData', 0, 'VData',-y(3,k));
        set(ray2, 'XData', x(1,k), 'YData', -x(2,k), 'UData', y(4,k)*sin(alpha), 'VData',-y(4,k)*cos(alpha));
        set(D_est, 'XData', x(1,k), 'YData', -x(2,k), 'UData', x_hat(1,k), 'VData',-x_hat(2,k));
        %set(D_dot_est, 'XData', x(1,k), 'YData', -x(2,k), 'UData', 30*x_hat(3,k), 'VData',-30*x_hat(4,k));
        pause(0.01);
    end
end

% trim data to loose initial point
x = x(:,2:end);
real_d = real_d(:,2:end);
x_hat = x_hat(:,2:end);
y = y(:,2:end);
y_unfilt = y_unfilt(:,2:end);
t = t(:,2:end);
V_ref = V_ref(:,2:end);
debug = debug(:,2:end);
% compute the true D_dot
real_d_dot = [0, diff(real_d(1,:))./diff(t);
              0, diff(real_d(2,:))./diff(t)];


%% === PLOTS ===

close all
fig=figure();
set(fig, 'Position', [100, 100, 800, 600])
% AUV trajectory
subplot('Position', [0.1, 0.5, 0.8, 0.4]);
hold on; grid on; 
set(gca, 'YDir', 'reverse')
plot(linspace(0,x(1,end)), terrain(linspace(0,x(1,end))), 'LineWidth', 2, "Color",[0/255,0/255,0]);
plot(x(1,:), x(2,:),'--','LineWidth',2,Color=[0.5,0.5,0.5])
num_snapshots = 3;
timestamps = round(linspace(100, size(x, 2) - 100, num_snapshots));
timestamps = [100,600,900];
img = imread('bluerov.jpeg');
[img_height, img_width, ~] = size(img);
auv_length=1.3;
auv_width=1.3;
image_scale_x = auv_length / 2 / (img_width / 100);
image_scale_y = auv_width / 2 / (img_height / 100);
for i = 1:length(timestamps)
    k = timestamps(i);
    image('XData', [x(1, k) - auv_length/2, x(1, k) + auv_length/2], 'YData', [x(2, k) - auv_width/2, x(2, k) + auv_width/2], 'CData', imresize(img, [round(img_height * image_scale_y), round(img_width * image_scale_x)]));

    fixed_arrowhead_size = 1;  
    quiver(x(1, k), x(2, k), 0, y(3, k), 0, 'r', 'LineWidth', 1.5, "Color", [193/255, 125/255, 17/255, 0.2], 'MaxHeadSize', fixed_arrowhead_size / sqrt(y(3, k)^2));
    quiver(x(1, k), x(2, k), y(4, k) * sin(alpha), y(4, k) * cos(alpha), 0, 'g', 'LineWidth', 1.5, "Color", [193/255, 125/255, 17/255, 0.2], 'MaxHeadSize', fixed_arrowhead_size / sqrt(y(4, k)^2));
    quiver(x(1, k), x(2, k), x_hat(1, k), x_hat(2, k), 0, 'b', 'LineWidth', 2, "Color", [52/255, 101/255, 164/255, 0.5], 'MaxHeadSize', fixed_arrowhead_size / sqrt(x_hat(1, k)^2 + x_hat(2, k)^2));
end
xlim([0,x(1,end)]);
legend({"Terrain", "Trajectory", "echosounder beams", "", "$\hat{D}$"}, 'Interpreter', 'latex', 'Location', 'best')
title("AUV Trajectory", 'Interpreter', 'latex');
ylabel("Z [m]", 'Interpreter', 'latex')
xlabel("X [m]", 'Interpreter', 'latex')

% error plot
subplot('Position', [0.1, 0.1, 0.8, 0.2]); hold on;
plot(t, d_ref- vecnorm(real_d), 'LineWidth', 2, 'Color',[0,0,0] );
title("Tracking Error", 'Interpreter', 'latex');
ylabel("$e$ [m]", 'Interpreter', 'latex')
xlabel("Time [s]", 'Interpreter', 'latex') 
grid on;
exportgraphics(gcf, 'imgs/' + output_folder_name +'/' + 'trajectory_error.pdf');

fig = figure();
set(fig, 'Position', [100, 100, 800, 600])
subplot(2,1,1); hold on
plot(t, real_d(1,:), 'LineWidth', 4, "Color" ,[52/255, 101/255, 164/255])
plot(t, real_d(2,:), 'LineWidth', 4,"Color" ,"#b36b00")
plot(t, x_hat(1,:), 'LineWidth', 2,"Color" ,"#6492ce")
plot(t, x_hat(2,:), 'LineWidth', 2,"Color" ,"#ffad33")%[193/255, 125/255, 17/255])
grid on;
ylabel("Position [m]", 'Interpreter', 'latex')
xlabel("Time [s]", 'Interpreter', 'latex') % Corrected 'm' to 's' for time
legend({"$D_x$", "$D_z$", "$\hat{D}_x$", "$\hat{D}_z$"}, 'Interpreter', 'latex', 'Location', 'best')
title("$D$ estimation", 'Interpreter', 'latex');
xlabel("Time [s]", 'Interpreter', 'latex')

subplot(2,1,2); hold on;
plot(t, real_d_dot(1,:), 'LineWidth', 4,"Color" ,[52/255, 101/255, 164/255])
plot(t, real_d_dot(2,:), 'LineWidth', 4,"Color" ,"#b36b00")
plot(t, x_hat(3,:), 'LineWidth', 2,"Color" ,"#6492ce")
plot(t, x_hat(4,:), 'LineWidth',2,"Color" ,"#ffad33")%[193/255, 125/255, 17/255])
grid on;
ylim([-0.3,0.3]);
ylabel("Velocity [m/s]", 'Interpreter', 'latex')
xlabel("Time [s]", 'Interpreter', 'latex') % Corrected 'm' to 's' for time
legend({"$\dot{D}_x$", "$\dot{D}_z$", "$\hat{\dot{D}}_x$", "$\hat{\dot{D}}_z$"}, 'Interpreter', 'latex')
title("$\dot{D}$ estimation", 'Interpreter', 'latex');
exportgraphics(gcf, 'imgs/' + output_folder_name +'/' + 'd_estimator.pdf');

% outlier rejection 
fig = figure();
set(fig, 'Position', [100, 100, 800, 600])
subplot(2,1,1); hold on; grid on;
plot(t, y_unfilt(1,:),"*");
plot(t, y(3,:),LineWidth=2.5);
legend({"$h_1$", "$h_1^{filtered}$"}, 'Interpreter', 'latex')
title("Vertical Echosounder", 'Interpreter', 'latex');
subplot(2,1,2); hold on; grid on;
plot(t, y_unfilt(2,:),"*"); 
plot(t, y(4,:),LineWidth=2.5);
title("Angled Echosounder", 'Interpreter', 'latex');
legend({"$h_2$", "$h_2^{filtered}$"}, 'Interpreter', 'latex')
xlabel("Time [s]", 'Interpreter', 'latex')
ylabel("Measurement [m]", 'Interpreter', 'latex')
exportgraphics(gcf, 'imgs/' + output_folder_name +'/' + 'outlier_rejection.pdf');

% inner-loop analysis
fig = figure();
set(fig, 'Position', [100, 100, 800, 600])
subplot(2,1,1); hold on; grid on;
plot(t, V_ref(1,:),'LineWidth', 4, "Color" ,[52/255, 101/255, 164/255]);
plot(t, x(3,:),'LineWidth', 2,"Color" ,"#6492ce");
title("Surge velocity tracking", 'Interpreter', 'latex');
legend({"$V_x^{ref}$", "$u$"}, 'Interpreter', 'latex')
xlabel("Time [s]", 'Interpreter', 'latex')
ylabel("Velocity [m/s]", 'Interpreter', 'latex')
subplot(2,1,2); hold on; grid on;
title("Heave velocity tracking", 'Interpreter', 'latex');
plot(t, V_ref(2,:), 'LineWidth', 4,"Color" ,"#b36b00");
plot(t, x(4,:),'LineWidth', 2,"Color" ,"#ffad33");
legend({"$V_z^{ref}$", "$w$"}, 'Interpreter', 'latex')
xlabel("Time [s]", 'Interpreter', 'latex')
ylabel("Velocity [m/s]", 'Interpreter', 'latex')
exportgraphics(gcf, 'imgs/' + output_folder_name +'/' +'inner_loops.pdf');