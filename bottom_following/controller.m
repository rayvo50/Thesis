function [tau,V] = controller(xhat, d_ref, U, X, surge_pid, heave_pid)
    % Implements a simple non-linear controller based on a simple Lyapunov 
    % theory to follow the desired distance to the terrain. See notes for 
    % more information

    % Implement Kinematic Controller
    Kp =1;                      % controller gain 
    D = xhat(1:2);              % extract D
    S = [xhat(2); -xhat(1)];    % Compute S by rotating D 90º anti-clockwise (from z to x)
    S = S/norm(S);              % normalize it
    e = (d_ref - norm(D));      % copmute error
    ksi = S - Kp*e*D;           % desired velocity
    V = U*ksi/norm(ksi);        % normalized velocity
    
    % send kinematic controller references to the inner-loops (Delta PIDs)  
    surge_pid.compute(X(3), V(1));
    heave_pid.compute(X(4), V(2));

    % pack tau for output
    tau = [surge_pid.output;
           heave_pid.output];
    
end