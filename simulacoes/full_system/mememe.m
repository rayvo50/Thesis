function u_sat = computeCommandYaw(obj, yaw, yaw_rate, yaw_ref, duration, frequency)
    % Initialisation parameters
    N_r = -0.5;
    I_z = 0.24;
    u_max = 5.0; % N.m
    u_min = -5.0; % N.m
    a = 10.0; % rad/s

    alpha = 1.0 / I_z;
    beta = -N_r / I_z;

    w_n = 0.5; % rad/s
    qsi = 0.7;
    pole = -4;

    % delta calculation
    delta = 1.0 / frequency;

    K_r = (2*qsi*w_n - pole - beta) / alpha;
    K_p = w_n * (w_n - 2*pole*qsi) / alpha;
    K_i = (- w_n^2 * pole) / alpha;
    k_a = 1.0 / delta;

    A = exp(-a*delta);
    B = 1 - A;

    % Change from degrees to radians
    yaw = deg2rad(yaw);
    yaw_rate = deg2rad(yaw_rate);
    yaw_ref = deg2rad(yaw_ref);

    % Compute control input
    error = wrapToPi(yaw_ref - yaw);

    if obj.first_it
        yaw_rate_dot = 0;
        yaw_dot = 0;
    else
        yaw_rate_dot = (yaw_rate - obj.yaw_rate_prev) / delta;
        yaw_dot = wrapToPi(yaw - obj.yaw_prev) / delta;
    end

    g = K_r * yaw_rate_dot + K_p * yaw_dot;
    g_filter = A * obj.g_filter_prev + g * B;

    u_d = K_i * error - g_filter;
    u_dot = u_d - k_a * (obj.u_prev - obj.u_sat_prev);
    u = obj.u_prev + u_dot * delta;

    if u < u_min
        u_sat = u_min;
    elseif u > u_max
        u_sat = u_max;
    else
        u_sat = u;
    end

    % Update new prev values
    obj.yaw_rate_prev = yaw_rate;
    obj.yaw_prev = yaw;
    obj.g_filter_prev = g_filter;
    obj.u_prev = u;
    obj.u_sat_prev = u_sat;

    obj.first_it = false;

end