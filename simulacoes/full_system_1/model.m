function x_new = model(x,u,Dt)
    % parameters
    r_max = rad2deg(0.5);
    u_rate = 0.05;
    
    x_new = zeros(5,1);
    x_new(1) = x(1) + x(3)*cosd(x(4));
    x_new(2) = x(2) + x(3)*sind(x(4));
    % surge "dynamic"
    if abs(u(1)-x(3)) <= u_rate*Dt
        x_new(3) = u(1);
    elseif u(1)-x(3) > u_rate*Dt
        x_new(3) = x(3) + u_rate*Dt;  
    elseif u(1)-x(3) < -u_rate*Dt
        x_new(3) = x(3) - u_rate*Dt;
    end
    % yaw "dynamic"
    if abs(u(2)-x(4)) <= r_max*Dt
        x_new(4) = u(2);
    elseif u(2)-x(4) > r_max*Dt
        x_new(4) = x(4) + r_max*Dt;
    elseif u(2)-x(4) < -r_max*Dt
        x_new(4) = x(4) - r_max*Dt;
    end
    % "yaw_rate" 
    x_new(5) = (x_new(4)-x(4))/Dt;
end