%% PLOTS 
close all;
load("results_fa_nc.mat")
x1 = x;
x_hat1 = x_hat;
u1=u;
y1=y;
t1 = t;
state1 = state;
load("results_fa_c.mat")
x2 = x;
x_hat2 = x_hat;
u2=u;
y2=y;
t2 = t;
Pd = [10,10,deg2rad(90)];
state2 = state;

% XY plot
fig = figure(); hold on; grid on;
% docking station for legend
plot(-999,-999, 's', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black', 'MarkerSize', 10);
% homing point
plot(0,20, 'x', 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red', 'MarkerSize', 10, 'LineWidth',2);
% path 
plot([0,0], [0,20], '--', 'Color',[0.5,0.5,0.5], LineWidth=2);
% No currents
xy1 = Rot(-Pd(3))*(x1(1:2,:) - Pd(1:2)'); plot(xy1(2,:), xy1(1,:), Color=[0 0.4470 0.7410], LineWidth=2)
% Currents
xy2 = Rot(-Pd(3))*(x2(1:2,:) - Pd(1:2)'); plot(xy2(2,:), xy2(1,:), Color=[ 0.4980 0.6509 0.3216], LineWidth=2)
% draw DS
plotFilledTriangle([0,0], .5, 0, 'black');
% draw auv at given positions
nr_samples = 8;
samples_time = [t(1:int32(length(t)/(nr_samples-1)):end-40)]; 
a = find(ismember(t, samples_time) == 1);
samples_mvector = [xy1(2,a); xy1(1,a); x1(3,a)-Pd(3)].';        
for i = 1:length(samples_mvector)
    GTF_Simulink_PlotAUV([samples_mvector(i,1),samples_mvector(i,2),0], [0,0,-samples_mvector(i,3)*180/pi-90], 0.1, 0, [0.9290 0.6940 0.1250],1);
end
% draw auv at given positions
nr_samples = 7;
samples_time = [t(1:int32(length(t)/(nr_samples-1)):end-40)]; 
a = find(ismember(t, samples_time) == 1);
samples_mvector = [xy2(2,a); xy2(1,a); x2(3,a)-Pd(3)].';        
for i = 1:length(samples_mvector)
    GTF_Simulink_PlotAUV([samples_mvector(i,1),samples_mvector(i,2),0], [0,0,-samples_mvector(i,3)*180/pi-90], 0.1, 0, [0.9333,0.0275,0],1);
end
drawArrow([12,2],0.5,-90,[ 0.4980 0.6509 0.3216])
text(12, 4, 'Current = 0.2 m/s', 'HorizontalAlignment', 'center', 'Color', [ 0.4980 0.6509 0.3216], 'FontSize', 12);
%labels and shit
xlabel('$y^{\mathcal{D}}$ [m]', 'Interpreter','latex'); ylabel('$x^{\mathcal{D}}$ [m]','Interpreter','latex');
legend('Docking Station', 'Homing Point','Reference Trajectory','No Current','Current 0.2 m/s', 'Location', 'best'); 

xlim([-2 20.5])
ylim([-2 20.5])
%axis equal
set(gcf, 'Position', [100, 100, 600, 600]); 
print(fig, '../../imagens_pic/xy_fa.png', '-dpng', '-r300'); 


% heading plot
fig = figure();hold on; grid on;
yaw1 = wrapTo360(rad2deg(x1(3,:)-Pd(3)));
yaw2 = wrapTo360(rad2deg(x2(3,:)-Pd(3)));
plot(t1,yaw1, Color=[0 0.4470 0.7410], LineWidth=2) 
plot(t2,yaw2, Color=[0.4980 0.6509 0.3216], LineWidth=2)
legend('No Current','Current 0.2 m/s', 'Location', 'best'); 
xlim([min(t), max(t) ]);
ylim([min(min(yaw1),min(yaw2))-2, max(max(yaw1),max(yaw2))+2]);
xlabel('Time [s]', 'Interpreter','latex'); ylabel('Heading \psi [º]');
set(gcf, 'Position', [100, 100, 600, 200]); 
print(fig, '../../imagens_pic/yaw_fa.png', '-dpng', '-r300'); 

% veolicty plot
fig = figure();hold on; grid on;
plot(t1, x1(4,:), Color=[0 0.4470 0.7410], LineWidth=2) 
plot(t2, x2(4,:), Color=[0.3010 0.7450 0.9330], LineWidth=2) 
plot(t1, x1(5,:), Color=[0.7647 0.0549 0], LineWidth=2)
plot(t2, x2(5,:), Color=[0.8500 0.3250 0.0980], LineWidth=2) 
legend('Surge, No current','Surge, Current','Sway, No current','Sway, Current', 'Location', 'best'); 
xlim([min(t), max(t) ]);
xlabel('Time [s]', 'Interpreter','latex'); ylabel('Speed [m/s]');
set(gcf, 'Position', [100, 100, 600, 200]); 
print(fig, '../../imagens_pic/uv_fa.png', '-dpng', '-r300'); 


% filter errors
fig = figure();hold on; grid on;
% position
yyaxis left;
error = sqrt((x_hat1(1,2:end)-xy1(1,2:end)).^2 + (x_hat1(2,2:end)-xy1(2,2:end)).^2 ) ;
plot(t1(2:end),error, Color=[0 0.4470 0.7410], LineWidth=2) 
ylabel('Position Error ($\hat{P} - P$) [m]', 'Interpreter', 'latex');
ylim([min(error)-0.1, max(error)+0.1]);
set(gca, 'ycolor', [0 0.4470 0.7410]); 
% heading
yyaxis right;
error = 180/pi*wrapToPi(x_hat1(5,3:end)-x1(3,3:end)+Pd(3));
plot(t1(3:end),error, Color=[0.8500 0.3250 0.0980], LineWidth=2) 
ylabel('Heading Error ($\hat{\psi} - \psi$) [$^\circ$]', 'Interpreter', 'latex');
ylim([min(error)-2, max(error)+2]);
set(gca, 'ycolor', [0.8500 0.3250 0.0980]); 
xlabel('Time [s]', 'Interpreter','latex');
xlim([min(t), max(t) ])
set(gcf, 'Position', [100, 100, 600, 200]);
print(fig, '../../imagens_pic/filter_error_fa.png', '-dpng', '-r300'); 


% cross track error
fig = figure();hold on; grid on;
a = find(state==2);
error = xy1(2,a);
plot(t1(a),error, Color=[0,0,0], LineWidth=2)
error = xy2(2,a);
plot(t2(a),error, Color=[0.7647 0.0549 0], LineWidth=2)
legend('No Current','Current 0.2 m/s', 'Location', 'best'); 
xlim([min(t1(a)), max(t1(a)) ]);
xlabel('Time [s]', 'Interpreter','latex'); ylabel('Cross-track Error [m]','Interpreter','latex');
set(gcf, 'Position', [100, 100, 600, 200]); 
print(fig, '../../imagens_pic/cross_track_fa.png', '-dpng', '-r300'); 

% heading error
fig = figure();hold on; grid on;
a = find(state==2);
error1 = 180/pi*wrapToPi(pi-x2(3,a)+Pd(3));
plot(t2(a),error1, Color=[0,0,0], LineWidth=2) 
error2 = 180/pi*wrapToPi(pi-x1(3,a)+Pd(3));
plot(t1(a),error2, Color=[0.7647 0.0549 0], LineWidth=2) 
legend('No Current','Current 0.2 m/s', 'Location', 'best'); 
ylim([min(min(error1),min(error2))-2, max(max(error1),max(error2))+2]);
xlim([min(t1(a)), max(t1(a)) ]);
xlabel('Time [s]', 'Interpreter','latex'); ylabel('Orientation Error [º]','Interpreter','latex');
set(gcf, 'Position', [100, 100, 600, 200]); 
print(fig, '../../imagens_pic/alignment_error_fa.png', '-dpng', '-r300'); 

% % control errors ig
% fig = figure();hold on; grid on;
% a = find(state==2);
% % position
% yyaxis left;
% error = xy2(2,a);
% plot(t2(a),error, Color=[0.1059 0.3686 0.1255], LineWidth=2)
% ylabel('Cross-track Error [m]','Interpreter','latex');
% set(gca, 'ycolor', [0.1059 0.3686 0.1255]); 
% % heading
% yyaxis right;
% error = 180/pi*wrapToPi(pi-x2(3,a)+Pd(3));
% plot(t2(a),error, Color=[0.6350 0.0780 0.1840], LineWidth=2) 
% ylim([min(error), max(error)+2])
% ylabel('Heading Error [º]');
% set(gca, 'ycolor', [0.6350 0.0780 0.1840]);
% xlim([min(t(a)), max(t(a)) ])
% xlabel('Time [s]', 'Interpreter','latex');
% set(gcf, 'Position', [100, 100, 600, 150]); 
% print(fig, '../../imagens_pic/docking_error_fa_c.png', '-dpng', '-r300'); 
























% a = find(state==2);
% error = xy1(2,a);
% plot(t1(a),error, Color=[0.1059 0.3686 0.1255], LineWidth=2)
% ylabel('Cross-track Error [m]','Interpreter','latex');
% set(gca, 'ycolor', [0,0,0]); 
% % heading
% yyaxis right;
% error = 180/pi*wrapToPi(pi-x1(3,a)+Pd(3));
% plot(t1(a),error, Color=[0.6350 0.0780 0.1840], LineWidth=2) 
% ylim([min(error), max(error)+2])
% ylabel('Heading Error [º]');
% set(gca, 'ycolor', [0.7647 0.0549 0]);
% xlim([min(t(a)), max(t(a)) ])
% xlabel('Time [s]', 'Interpreter','latex');
% set(gcf, 'Position', [100, 100, 600, 150]); 
% print(fig, '../../imagens_pic/docking_error_fa_nc.png', '-dpng', '-r300'); 
