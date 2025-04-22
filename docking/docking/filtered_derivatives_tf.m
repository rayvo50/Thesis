function [dx, d2x] = filtered_derivatives_tf(t, x, fc)
    % Compute first and second derivatives using low-pass filtered transfer functions.
    % Ensures unit gain for proper scaling.
    %
    % Inputs:
    % - t: time vector
    % - x: signal (column vector)
    % - fc: cutoff frequency (Hz) for filtering
    %
    % Outputs:
    % - dx: first derivative (filtered)
    % - d2x: second derivative (filtered)

    dt = mean(diff(t)); % Time step
    wc = 2 * pi * fc; % Convert cutoff to rad/s

    % Define transfer functions with proper gain correction
    s = tf('s');
    H1 = (wc * s) / (wc + s);  % First derivative (normalized)
    H2 = (wc^2 * s^2) / ((wc + s)^2); % Second derivative (normalized)

    % Convert to discrete-time (Tustin)
    Hd1 = c2d(H1, dt, 'tustin'); 
    Hd2 = c2d(H2, dt, 'tustin'); 

    % Apply filtering
    dx = lsim(Hd1, x, t);
    d2x = lsim(Hd2, x, t);
end
