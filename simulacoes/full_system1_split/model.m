function x_new = model(x,u,Dt)
    % parameters
    r_max = rad2deg(0.5);
    u_rate = 0.05;
    
    x_new = zeros(4,1);
    x_new(1) = x(1) + x(4)*cosd(x(3));
    x_new(2) = x(2) + x(4)*sind(x(3));

    %yaw "dynamic"
    if abs(u(2)-x(3)) <= r_max*Dt
        x_new(3) = u(2);
    elseif u(2)-x(3) > r_max*Dt
        x_new(3) = x(3) + r_max*Dt;
    elseif u(2)-x(3) < -r_max*Dt
        x_new(3) = x(3) - r_max*Dt;
    end

    % surge "dynamic"
    if abs(u(1)-x(4)) <= u_rate*Dt
        x_new(4) = u(1);
    elseif u(1)-x(4) > u_rate*Dt
        x_new(4) = x(4) + u_rate*Dt;  
    elseif u(1)-x(4) < -u_rate*Dt
        x_new(4) = x(4) - u_rate*Dt;
    end
    
end