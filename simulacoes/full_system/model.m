function x_new = model(x,u,Dt, Vc)
    % parameters
    K=1;
    tau_surge = 2;
    tau_yaw = 2;
    
    x_new = zeros(4,1);
    x_new(3) = x(3) + (K*Dt/tau_yaw) *wrapTo180(u(2) - x(3));
    x_new(4) = x(4) + (Dt/tau_surge) * (K * u(1) - x(4));
    x_new(1) = x(1) + (x_new(4)*cosd(x_new(3))+Vc(1))*Dt;
    x_new(2) = x(2) + (x_new(4)*sind(x_new(3))+Vc(2))*Dt;
    

end