%% LOAD DATA
clear;
close all;

simulation = false;

if (simulation == false)
    % multiple_trackers = false; % one tracker
    multiple_trackers = true; % two trackers
    
    if (multiple_trackers == false) % one tracker
        load('slap/2023-02-13/mred_cut.mat');
        load('slap/2023-02-13/mvector_2_mission_extended_2min.mat');
        load('slap/2023-02-13/mblack_cut.mat');
    else % two trackers
        load('2023-11-10/mblack__2023-11-10-16-35-30_cut.mat')
        load('2023-11-10/mred__2023-11-10-16-35-02_cut.mat')
        load('2023-11-10/mvector_mission.mat')
    end
else
    % load manually
end
%% get time (internal/pc clock) differences between vehicles

if (simulation == false)
    mvector = struct;
    mvector.idx = floor(length(mvector0_drivers_gps.utc_time)/2);
    mvector.t = mvector0_drivers_gps.t(mvector.idx);
    mvector.utc_time = mvector0_drivers_gps.utc_time(mvector.idx);
    
    % mblack
    mblack =  struct;
    [placeholder, mblack.idx] = min(abs(mblack0_drivers_gps.utc_time - mvector.utc_time)); % index corresponding to same utc_time as in mvector
    mblack.delta_to_mvector = mvector.t - mblack0_drivers_gps.t(mblack.idx); % sum this value to mblack t (internal clock) to synchronise with mvector internal clock (t)
    
    % mred
    mred =  struct;
    [placeholder_, mred.idx] = min(abs(mred0_drivers_gps.utc_time - mvector.utc_time)); % index corresponding to same utc_time as in mvector
    mred.delta_to_mvector = mvector.t - mred0_drivers_gps.t(mred.idx); % sum this value to mred t (internal clock) to synchronise with mvector internal clock (t)
else
    % default values for simulation
    mvector = struct;
    mblack =  struct;
    mblack.delta_to_mvector = 0;
    mred =  struct;
    mred.delta_to_mvector = 0;
end
%% compute T0 (initial time for SLAP)

if (simulation == false)
    if (multiple_trackers == false) % one tracker
        cut = 4500;
        t0 = mvector0_nav_filter_state.t(cut);
    else % two trackers
        t0_mblack = mblack0_slap_dekf_etc_info.t(1);
        t0_mred = mred0_slap_dekf_etc_info.t(1);
    
        t0 = min([t0_mblack + mblack.delta_to_mvector, t0_mred + mred.delta_to_mvector]);
        [value, cut] = min(abs(mvector0_nav_filter_state.t - t0));
    end
else
    t0 = 0;
end

%% TRAJECTORIES
if (simulation == false)
    if (multiple_trackers == false) % one tracker
    
        x0 = mblack0_nav_filter_state.easting(cut);
        y0 = mblack0_nav_filter_state.northing(cut);
        
        f1 = figure("Name", "Trajectories");
        plot(mblack0_nav_filter_state.easting(cut:end) - x0, mblack0_nav_filter_state.northing(cut:end) - y0, 'k-', ...
             mvector0_nav_filter_state.easting(cut:end) - x0, mvector0_nav_filter_state.northing(cut:end) - y0, 'y-', ...
             mblack0_slap_st_curve.traj_pos(cut:end,2) - x0, mblack0_slap_st_curve.traj_pos(cut:end,1) - y0, 'k--', ...
             'LineWidth',2);
        xlabel("Easting [m]", 'FontSize', 15);
        ylabel("Northing [m]",'FontSize', 15);
        xticks([-10 80]);
        yticks([-25 40]);
        xlim([-10 80]);
        ylim([-25 40]);
        ax = gca;
        ax.FontSize = 15;
        axis equal;
    else % two trackers
        % cut = 3500;  
        % t0 = mvector0_nav_filter_state.t(cut);
    
        x0 = mblack0_nav_filter_state.easting(cut);
        y0 = mblack0_nav_filter_state.northing(cut);
        
        f1 = figure("Name", "Trajectories");
        plot(mblack0_nav_filter_state.easting(cut:end) - x0, mblack0_nav_filter_state.northing(cut:end) - y0, 'k-', ...
             mred0_nav_filter_state.easting(cut:end) - x0, mred0_nav_filter_state.northing(cut:end) - y0, 'r-', ...
             mvector0_pathdata.pd(:,2) - x0, mvector0_pathdata.pd(:,1) - y0, 'b-', ...
             mblack0_slap_st_curve.traj_pos(cut:end,2) - x0, mblack0_slap_st_curve.traj_pos(cut:end,1) - y0, 'k--', ...
             mred0_slap_st_curve.traj_pos(cut:end,2) - x0, mred0_slap_st_curve.traj_pos(cut:end,1) - y0, 'r--', ...
             'LineWidth',1);
        
        hold on;

        % images of the vehicles

        % get time instants to get snaps from vehicles
        nr_samples = 9;
        sample_length = length(mvector0_nav_filter_state.t);
        samples_time = [mvector0_nav_filter_state.t(1:sample_length/(nr_samples-1):end) mvector0_nav_filter_state.t(end)];

        % samples mvector
        a = find(ismember(mvector0_nav_filter_state.t, samples_time) == 1);
        samples_mvector = [mvector0_nav_filter_state.easting(a) - x0 ; mvector0_nav_filter_state.northing(a) - y0; mvector0_nav_filter_state.yaw(a)].';
        
        % samples mblack
        a = zeros(1, length(samples_time));
        for i = 1:length(samples_time)
            diffs = abs(mblack0_nav_filter_state.t + mblack.delta_to_mvector - samples_time(i));
            [~, index] = min(diffs);
            a(i) = index;
        end
        samples_mblack = [mblack0_nav_filter_state.easting(a) - x0; mblack0_nav_filter_state.northing(a) - y0; mblack0_nav_filter_state.yaw(a)].';

        % samples mred
        a = zeros(1, length(samples_time));
        for i = 1:length(samples_time)
            diffs = abs(mred0_nav_filter_state.t + mred.delta_to_mvector - samples_time(i));
            [~, index] = min(diffs);
            a(i) = index;
        end
        samples_mred = [mred0_nav_filter_state.easting(a) - x0; mred0_nav_filter_state.northing(a) - y0; mred0_nav_filter_state.yaw(a)].';
        
        % plot triangles
        for i = 1:length(samples_mvector)
            % plotFilledTriangle(samples_mvector(i,1:2), 4.5, samples_mvector(i,3), 'yellow');
            GTF_Simulink_PlotAUV([samples_mvector(i,1),samples_mvector(i,2),0], [0,0,-samples_mvector(i,3)-90], 0.5, 0, [255,255,153]/255,1);
            hold on;
        end

        for i = 1:length(samples_mred)
            % plotFilledTriangle(samples_mred(i,1:2), 4.5, samples_mred(i,3), 'red');
            GTF_Simulink_PlotAUV([samples_mred(i,1),samples_mred(i,2),0], [0,0,-samples_mred(i,3)-90], 0.5, 0, [255,102,0]/255,1);
            hold on;
        end

        for i = 1:length(samples_mblack)
            % plotFilledTriangle(samples_mblack(i,1:2), 4.5, samples_mblack(i,3), 'black');
            GTF_Simulink_PlotAUV([samples_mblack(i,1),samples_mblack(i,2),0], [0,0,-samples_mblack(i,3)-90], 0.5, 0, [50,50,50]/255,1);
            hold on;
        end
        
        text(-84,-120, "End"); hold on;
        text(5,-5, "Start"); hold on;
        text(-48.5,-23, "Change of"); hold on;
        text(-48,-30, "formation"); hold on;


        xlabel("Easting [m]", 'FontSize', 15);
        ylabel("Northing [m]",'FontSize', 15);
        % xticks([-100 30]);
        % yticks([-100 0]);
        xlim([-110 50]);
        ylim([-130 5]);
        ax = gca;
        ax.FontSize = 15;
        % axis equal;

        legend('$\mbox{\boldmath $\textrm{\textbf{p}}$}^{[1]}$', ...
               '$\mbox{\boldmath $\textrm{\textbf{p}}$}^{[2]}$', ...
               '$\mbox{\boldmath $\textrm{\textbf{q}}$}$', ...
               '$\hat{\mbox{\boldmath $\textrm{\textbf{q}}$}}^{[1]}$', ...
               '$\hat{\mbox{\boldmath $\textrm{\textbf{q}}$}}^{[2]}$', ...
               'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex', 'Location', 'east');

        exportgraphics(f1,'2023-11-10/_slap_trajectories.png','Resolution',500);
    end
else
    x0 = mblack0_nav_filter_state.easting(cut);
    y0 = mblack0_nav_filter_state.northing(cut);
    
    f1 = figure("Name", "Trajectories");
    plot(mblack0_nav_filter_state.easting(cut:end) - x0, mblack0_nav_filter_state.northing(cut:end) - y0, 'k-', ...
         mred0_nav_filter_state.easting(cut:end) - x0, mred0_nav_filter_state.northing(cut:end) - y0, 'r-', ...
         mvector0_nav_filter_state.easting(cut:end) - x0, mvector0_nav_filter_state.northing(cut:end) - y0, 'y-', ...
         mblack0_slap_st_curve.traj_pos(cut:end,2) - x0, mblack0_slap_st_curve.traj_pos(cut:end,1) - y0, 'k--', ...
         mred0_slap_st_curve.traj_pos(cut:end,2) - x0, mred0_slap_st_curve.traj_pos(cut:end,1) - y0, 'r:', ...
         'LineWidth',2);
    xlabel("Easting [m]", 'FontSize', 15);
    ylabel("Northing [m]",'FontSize', 15);
end

%% target state estimation error 

if (simulation == false)
    if (multiple_trackers == false) % one tracker
    
        mvector_state = [mvector0_nav_filter_state.t.', mvector0_nav_filter_state.northing.', mvector0_nav_filter_state.easting.'];
        mblack_est = [(mblack0_slap_st_curve.t + mblack.delta_to_mvector).', mblack0_slap_st_curve.traj_pos(:,1:2)];
    
        % mvector_state = mvector_state(cut:end);
        % mblack_est = mblack_est(cut:end);
        
        mblack_est_error = zeros(length(mblack_est(:,1)), 1);
    
        for i = 1:length(mblack_est_error)
            [value, idx] = min(abs(mvector_state(:,1)- mblack_est(i, 1)));
            mblack_est_error(i) = norm(mvector_state(idx, 2:3) - mblack_est(i, 2:3));
        end
        
        f2 = figure("Name", "Target State Estimation Error");
        plot(mblack_est(:,1) - t0, mblack_est_error, ...
             'LineWidth',2);
        xlabel("Time [s]", 'FontSize', 15);
        ylabel("Error [m]",'FontSize', 15);
        % xticks([-10 80]);
        % yticks([-25 40]);
        xlim([0 max(mblack_est(:,1) - t0)]);
        % ylim([-25 40]);
        ax = gca;
        ax.FontSize = 15;
    else % two trackers
    
        mvector_state = [mvector0_nav_filter_state.t.', mvector0_nav_filter_state.northing.', mvector0_nav_filter_state.easting.'];
        mblack_est = [(mblack0_slap_st_curve.t + mblack.delta_to_mvector).', mblack0_slap_st_curve.traj_pos(:,1:2)];
        mred_est = [(mred0_slap_st_curve.t + mred.delta_to_mvector).', mred0_slap_st_curve.traj_pos(:,1:2)];
    
        % mvector_state = mvector_state(cut:end);
        % mblack_est = mblack_est(cut:end);
        % mred_est = mred_est(cut:end);
        
        mblack_est_error = zeros(length(mblack_est(:,1)), 1);
        mred_est_error = zeros(length(mred_est(:,1)), 1);
    
        for i = 1:length(mblack_est_error)
            [value, idx] = min(abs(mvector_state(:,1)- mblack_est(i, 1)));
            mblack_est_error(i) = norm(mvector_state(idx, 2:3) - mblack_est(i, 2:3));
        end
    
        for i = 1:length(mred_est_error)
            [value, idx] = min(abs(mvector_state(:,1)- mred_est(i, 1)));
            mred_est_error(i) = norm(mvector_state(idx, 2:3) - mred_est(i, 2:3));
        end
        
        f2 = figure("Name", "Target State Estimation Errors");
        plot(mblack_est(:,1) - t0, mblack_est_error, 'k-',...
             mred_est(:,1) - t0, mred_est_error, 'r-', ...
             'LineWidth',2);
        xlabel("$t[s]$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
        ylabel("Errors [m]",'FontSize', 20);
        % xticks([-10 80]);
        % yticks([-25 40]);
        xlim([0 max([mblack_est(end,1) - t0, mred_est(end,1) - t0])]);
        % ylim([-25 40]);
        ax = gca;
        ax.FontSize = 20;
        legend('$||\tilde{ \mbox{\boldmath $\textrm{\textbf{x}}$} }^{[1]}(t)||$','$||\tilde{ \mbox{\boldmath $\textrm{\textbf{x}}$} }^{[2]}(t)||$', 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex', 'Location', 'northeast');   

        f2.Position = [100 100 1000 300];
        exportgraphics(f2,'2023-11-10/_slap_target_state_estimation_error.png','Resolution',500);
    end
else
    mvector_state = [mvector0_nav_filter_state.t.', mvector0_nav_filter_state.northing.', mvector0_nav_filter_state.easting.'];
    mblack_est = [(mblack0_slap_st_curve.t + mblack.delta_to_mvector).', mblack0_slap_st_curve.traj_pos(:,1:2)];
    mred_est = [(mred0_slap_st_curve.t + mred.delta_to_mvector).', mred0_slap_st_curve.traj_pos(:,1:2)];

    % mvector_state = mvector_state(cut:end);
    % mblack_est = mblack_est(cut:end);
    % mred_est = mred_est(cut:end);
    
    mblack_est_error = zeros(length(mblack_est(:,1)), 1);
    mred_est_error = zeros(length(mred_est(:,1)), 1);

    for i = 1:length(mblack_est_error)
        [value, idx] = min(abs(mvector_state(:,1)- mblack_est(i, 1)));
        mblack_est_error(i) = norm(mvector_state(idx, 2:3) - mblack_est(i, 2:3));
    end

    for i = 1:length(mred_est_error)
        [value, idx] = min(abs(mvector_state(:,1)- mred_est(i, 1)));
        mred_est_error(i) = norm(mvector_state(idx, 2:3) - mred_est(i, 2:3));
    end
    
    f2 = figure("Name", "Target State Estimation Errors");
    plot(mblack_est(:,1) - t0, mblack_est_error, 'k-',...
         mred_est(:,1) - t0, mred_est_error, 'r-', ...
         'LineWidth',2);
    xlabel("Time [s]", 'FontSize', 15);
    ylabel("Error [m]",'FontSize', 15);
end



%% range measurements over time

if (simulation == false)

    if (multiple_trackers == false) % one tracker
    
        range = struct;
        range.t = mblack0_sensors_usbl_fix.t + mblack.delta_to_mvector;
        range.data = mblack0_sensors_usbl_fix.range;
        
        % condition = range.data ~= 0 & range.data < 100;
        condition = range.data ~= 0;
        
        f3 = figure("Name", "Range Measurements");
        plot(range.t(condition) - t0, range.data(condition), 'r-', ...
             'LineWidth',2);
        xlabel("Time [s]", 'FontSize', 15);
        ylabel("Range [m]",'FontSize', 15);
        % xticks([-10 80]);
        % yticks([-25 40]);
        xlim([0 max(range.t(condition) - t0)]);
        % ylim([-25 40]);
        ax = gca;
        ax.FontSize = 15;
    else % two trackers
    
        range_k = struct;
        range_k.t = mblack0_sensors_usbl_fix.t + mblack.delta_to_mvector;
        range_k.data = mblack0_sensors_usbl_fix.range;
        
        % condition_k = range_k.data ~= 0 & range_k.data < 100;
        condition_k = range_k.data ~= 0;
    
        range_r = struct;
        range_r.t = mred0_sensors_usbl_fix.t + mred.delta_to_mvector;
        range_r.data = mred0_sensors_usbl_fix.range;
        
        % condition_r = range_r.data ~= 0 & range_r.data < 100;
        condition_r = range_r.data ~= 0;
        
        f3 = figure("Name", "Range Measurements");
        plot(range_k.t(condition_k) - t0, range_k.data(condition_k), 'k-', ...
             range_r.t(condition_r) - t0, range_r.data(condition_r), 'r-', ...
             'LineWidth',2);
        hold on;
        % rectangle('Position', [1800 0 500 200], 'EdgeColor', 'k', 'LineWidth', 2);
        % hold on;
        q = quiver(2000, 100, 0, 250 ,0, 'LineWidth', 2, 'Color', 'black', 'MaxHeadSize', 1.5);

        xlabel("$t[s]$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
        ylabel("[m]",'FontSize', 20);
        % xticks([-10 80]);
        % yticks([-25 40]);
        xlim([0 max([range_k.t(condition_k) - t0, range_r.t(condition_r) - t0])]);
        % ylim([-25 40]);
        ax = gca;
        ax.FontSize = 20;
        legend('$y^{[1]}(t)$','$y^{[2]}(t)$', 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex', 'Location', 'north');

        % plot zoom section
        insetAxes = axes('Parent', f3);
        set(insetAxes, 'Position', [0.62, 0.45, 0.24, 0.4]);
        plot(insetAxes, range_k.t(condition_k) - t0, range_k.data(condition_k), 'k-', ...
                        range_r.t(condition_r) - t0, range_r.data(condition_r), 'r-', ...
                        'LineWidth',2);
        xlim([1800 2300]);
        ylim([0 12]);
        xlabel("$t[s]$", 'FontAngle', 'italic', 'Interpreter','latex');
        ylabel("[m]");
        title('Zoomed in section, $t \in [1800, \, 2300]s$', 'Interpreter','latex');
        
        
        f3.Position = [100 100 1000 300];
        exportgraphics(f3,'2023-11-10/_slap_range_measurements.png','Resolution',500);
    end
else
    range_k = struct;
    range_k.t = mblack0_sensors_usbl_fix.t + mblack.delta_to_mvector;
    range_k.data = mblack0_sensors_usbl_fix.range;
    
    % condition_k = range_k.data ~= 0 & range_k.data < 100;
    condition_k = range_k.data ~= 0;

    range_r = struct;
    range_r.t = mred0_sensors_usbl_fix.t + mred.delta_to_mvector;
    range_r.data = mred0_sensors_usbl_fix.range;
    
    % condition_r = range_r.data ~= 0 & range_r.data < 100;
    condition_r = range_r.data ~= 0;
    
    f3 = figure("Name", "Range Measurements");
    plot(range_k.t(condition_k) - t0, range_k.data(condition_k), 'k-', ...
         range_r.t(condition_r) - t0, range_r.data(condition_r), 'r-', ...
         'LineWidth',2);
    xlabel("Time [s]", 'FontSize', 15);
    ylabel("Range [m]",'FontSize', 15);
end

%% coordination states

if (simulation == false)

    if (multiple_trackers == true) % one tracker
        
        f4 = figure("Name", "Coordination State (Gamma)");
        plot(mblack0_slap_internal_gamma.t - t0 + mblack.delta_to_mvector, mblack0_slap_internal_gamma.gamma, 'k-', ...
             mred0_slap_internal_gamma.t - t0 + mred.delta_to_mvector, mred0_slap_internal_gamma.gamma, 'r-', ...
             'LineWidth',2);
        xlabel("$t[s]$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
        ylabel("",'FontSize', 20);
        % xticks([-10 80]);
        % yticks([-25 40]);
        xlim([0 max([mblack0_slap_internal_gamma.t - t0 + mblack.delta_to_mvector, mred0_slap_internal_gamma.t - t0 + mred.delta_to_mvector])]);
        % ylim([-25 40]);
        ax = gca;
        ax.FontSize = 20;
        legend('$\gamma^{[1]}$','$\gamma^{[2]}$', 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex', 'Location', 'northwest');

        f4.Position = [100 100 1000 200];
        exportgraphics(f4,'2023-11-10/_slap_gamma.png','Resolution',500);
    end
else
    f4 = figure("Name", "Coordination State (Gamma)");
    plot(mblack0_slap_internal_gamma.t - t0 + mblack.delta_to_mvector, mblack0_slap_internal_gamma.gamma, 'k-', ...
         mred0_slap_internal_gamma.t - t0 + mred.delta_to_mvector, mred0_slap_internal_gamma.gamma, 'r-', ...
         'LineWidth',2);
    xlabel("Time [s]", 'FontSize', 15);
    ylabel("Gamma",'FontSize', 15);
end
%% broadcast signals of the two trackers

if (simulation == false)

    if (multiple_trackers == true) % two trackers
        
        % COOPERATIVE
        
        f5 = figure("Name", "CPF Broadcast Signal");
        plot(mblack0_slap_cpf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_cpf_etc_info.broadcast_signal*2, 'k.', ...
             mred0_slap_cpf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_cpf_etc_info.broadcast_signal, 'r.', ...
             'LineWidth', 1, 'MarkerSize', 8);
        gca.FontSize = 15;
        xlabel("$t[s]$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
        ylabel("$t^{[i]}_n$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
        % xticks([-10 80]);
        yticks([]);
        %xlim([0 max([mblack0_slap_cpf_etc_info.t - t0 + mblack.delta_to_mvector, mred0_slap_cpf_etc_info.t - t0 + mred.delta_to_mvector])]);
        xlim([300 1400]);
        ylim([0.5 2.5]);
        f5.Position = [100 100 1000 200];
        exportgraphics(f5,'2023-11-10/_slap_CPF_broadcast_signal.png','Resolution',500);
    
        % DEKF
        
        f6 = figure("Name", "DEKF Broadcast Signal");
        plot(mblack0_slap_dekf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_dekf_etc_info.broadcast_signal*2, 'k.', ...
             mred0_slap_dekf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_dekf_etc_info.broadcast_signal, 'r.', ...
             'LineWidth',1, 'MarkerSize', 8);
        xlabel("$t[s]$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
        ylabel("$k^{[i]}_l$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
        % xticks([-10 80]);
        yticks([]);
        %xlim([0 max([mblack0_slap_dekf_etc_info.t - t0 + mblack.delta_to_mvector, mred0_slap_dekf_etc_info.t - t0 + mred.delta_to_mvector])]);
        xlim([300 1400]);
        ylim([0.5 2.5]);
        ax = gca;
        ax.FontSize = 60;
        f6.Position = [100 100 1000 200];
        exportgraphics(f6,'2023-11-10/_slap_DEKF_broadcast_signal.png','Resolution',500);
    end
else
    % COOPERATIVE
        
    f5 = figure("Name", "CPF Broadcast Signal");
    plot(mblack0_slap_cpf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_cpf_etc_info.broadcast_signal*2, 'k.', ...
         mred0_slap_cpf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_cpf_etc_info.broadcast_signal, 'r.', ...
         'LineWidth',1);
    xlabel("Time [s]", 'FontSize', 15);
    ylabel("CPF Broadcast Signal",'FontSize', 15);

    ax = gca;
    ax.FontSize = 15;

    % DEKF
    
    f6 = figure("Name", "DEKF Broadcast Signal");
    plot(mblack0_slap_dekf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_dekf_etc_info.broadcast_signal*2, 'k.', ...
         mred0_slap_dekf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_dekf_etc_info.broadcast_signal, 'r.', ...
         'LineWidth',1);
    xlabel("Time [s]", 'FontSize', 15);
    ylabel("DEKF Broadcast Signal",'FontSize', 15);

    ax = gca;
    ax.FontSize = 15;
end

%% KLD and Threshold

if (simulation == false)
    % DEKF
    f7 = figure("Name", "KLD and Threshold");
    subplot(2,1,1)
    plot(mblack0_slap_dekf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_dekf_etc_info.threshold, 'b', ...
         mblack0_slap_dekf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_dekf_etc_info.error, 'k', ...
         'LineWidth',1);
    xlabel("$t[s]$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
    xlim([300 1400]);
    ylim([0 6]);
    yticks([0.5 1 5]);
    legend('$g^{[1]}(t)$','$\mathcal{K}\mathcal{L}\mathcal{D}^{[1]}(t)$', 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex', 'Location', 'northwest');

    ax = gca;
    ax.FontSize = 20;

    % f8 = figure("Name", "KLD and Threshold");
    subplot(2,1,2)
    plot(mred0_slap_dekf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_dekf_etc_info.threshold, 'b', ...
         mred0_slap_dekf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_dekf_etc_info.error, 'r', ...
         'LineWidth',1);
    xlabel("$t[s]$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
    xlim([300 1400]);
    ylim([0 6]);
    yticks([0.5 1 5]);
    legend('$g^{[2]}(t)$','$\mathcal{K}\mathcal{L}\mathcal{D}^{[2]}(t)$', 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex', 'Location', 'northwest');

    ax = gca;
    ax.FontSize = 20;

    f7.Position = [100 100 1000 650];
    exportgraphics(f7,'2023-11-10/_slap_DEKF_kld_error.png','Resolution',500);
    
    % CPF
    f8 = figure("Name", "KLD and Threshold");
    subplot(2,1,1)
    plot(mblack0_slap_cpf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_cpf_etc_info.threshold, 'b', ...
         mblack0_slap_cpf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_cpf_etc_info.error, 'k', ...
                  'LineWidth',1);
    xlabel("$t[s]$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
    xlim([300 1400]);
    ylim([0 0.55]);
    yticks([0.01 0.1 0.5]);
    legend('$h^{[1]}(t)$','$||\tilde{\gamma}^{[1]}(t)||$', 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex', 'Location', 'northwest');

    ax = gca;
    ax.FontSize = 20;

    % f8 = figure("Name", "KLD and Threshold");
    subplot(2,1,2)
    plot(mred0_slap_cpf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_cpf_etc_info.threshold, 'b', ...
         mred0_slap_cpf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_cpf_etc_info.error, 'r', ...
         'LineWidth',1);
    xlabel("$t[s]$", 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex');
    xlim([300 1400]);
    ylim([0 0.55]);
    yticks([0.01 0.1 0.5]);
    legend('$h^{[2]}(t)$','$||\tilde{\gamma}^{[2]}(t)||$', 'FontAngle', 'italic', 'FontSize', 20, 'Interpreter','latex', 'Location', 'northwest');

    ax = gca;
    ax.FontSize = 20;

    f8.Position = [100 100 1000 650];
    exportgraphics(f8,'2023-11-10/_slap_CPF_kld_error.png','Resolution',500);

else
    f7 = figure("Name", "KLD and Threshold");
    plot(mblack0_slap_dekf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_dekf_etc_info.error, 'k', ...
         mblack0_slap_dekf_etc_info.t - t0 + mblack.delta_to_mvector, mblack0_slap_dekf_etc_info.threshold, 'b', ...
         'LineWidth',1);
    xlabel("Time [s]", 'FontSize', 15);
    ylim([0 2.5]);

    ax = gca;
    ax.FontSize = 15;

    f8 = figure("Name", "KLD and Threshold");
    plot(mred0_slap_dekf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_dekf_etc_info.error, 'r', ...
         mred0_slap_dekf_etc_info.t - t0 + mred.delta_to_mvector, mred0_slap_dekf_etc_info.threshold, 'b', ...
         'LineWidth',1);
    xlabel("Time [s]", 'FontSize', 15);
    ylim([0 2.5]);

    ax = gca;
    ax.FontSize = 15;
end

