function x_new = model(x,tau,Dt)
    % Implements the nonlinear model of the AUV using simplied equation
    % from Fossen et al 
    % Inputs:
    %   x - the previous state
    %   tau - the input (force aplied on the center of mass of the vehicle)
    %   Dt - Timestep
    
    m = 11.2;           % mass
    Xudot = 27.08;      % hydrodinamic added mass in x body
    Zwdot = 29.9081;    % hydrodinamic added mass in z body
    mu = m+Xudot;
    du = 0.1213 ;       % linear drag in x body
    duu = 23.9000;      % quadratic drag in x body
    mw = m+Zwdot;
    dw = 1.1130;        % linear drag in z body
    dww = 50.2780;      % quadratic drag in z body
    gw = 0;
   
    % unpack input forces
    tau_u = tau(1);    
    tau_w = tau(2);
    
    % TODO: add sea current effect
    % Vc_ = Rot(-x(3))*Vc;
    
    % Aply non linear dynamic model
    x_new(3) = x(3) + Dt * 1/mu * ( tau_u - (du * x(3) + duu *x(3)*abs(x(3))) );
    x_new(4) = x(4) + Dt * 1/mw * ( tau_w - (dw * x(4) + dww *x(4)*abs(x(4)) + gw) );
    % Aply dynamic model
    x_new(1) = x(1) + Dt * x_new(3);
    x_new(2) = x(2) + Dt * x_new(4);

end