function x_new = model(x,u,Dt)
    x_new = zeros(4,1);
    x_new(1) = x(1) + x(3)*cosd(x(4));
    x_new(2) = x(2) + x(3)*sind(x(4));
    if abs(u(2)-x(4)) <= r_max*Dt
        x_new(4) = u(2);
    elseif u(2)-x(4) > r_max*Dt
        x_new(4) = x(4) + r_max*Dt;
    elseif u(2)-x(4) < -r_max*Dt
        x_new(4) = x(4) - r_max*Dt;
    end
    if abs(u(1)-x(3)) <= u_rate*Dt
        x_new(3) = u(1);
    elseif u(1)-x(4) > u_rate*Dt
        x_new(3) = x(3) + u_rate*Dt;
    elseif u(1)-x(4) < -u_rate*Dt
        x_new(3) = x(3) - u_rate*Dt;
    end
end