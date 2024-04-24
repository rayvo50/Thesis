function x_new = model(x,u,Dt, Vc)
    % parameters
    K=1;
    tau_surge = 2;
    tau_yaw = 1;
    
    x_new = zeros(4,1);
    x_new(1) = x(1) + (u(1)*cosd(u(2))+Vc(1))*Dt;
    x_new(2) = x(2) + (u(1)*sind(u(2))+Vc(2))*Dt;
    
    x_new(3) = u(2);
    x_new(4) = u(1);

    % % "dYnAmIcS"
    % x_new(3) = x(3) + (Dt/tau_yaw) * (K * u(2) - x(3));
    % x_new(4) = x(4) + (Dt/tau_surge) * (K * u(1) - x(4));
    
    %sus
    x_new = real(x_new); 
end