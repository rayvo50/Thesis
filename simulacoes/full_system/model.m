function x_new = model(x,u,Dt, Vc, Pd)
    K=1;
    tau_surge = 2;
    tau_yaw = 2;

    if isnan(u(2))
        yaw_ref = u(3) + Pd(3);
    else 
        yaw_ref = u(2);
    end

    x_new = zeros(5,1);
    x_new(3) = x(3) + (K*Dt/tau_yaw) *wrapTo180(yaw_ref - x(3));
    x_new(4) = x(4) + (Dt/tau_surge) * (K * u(1) - x(4));
    x_new(1) = x(1) + (x_new(4)*cosd(x_new(3))+Vc(1))*Dt;
    x_new(2) = x(2) + (x_new(4)*sind(x_new(3))+Vc(2))*Dt;
    x_new(5) = (x_new(3)-x(3))/Dt;

end