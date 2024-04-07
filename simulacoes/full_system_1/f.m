function x_p = f(x,u,Dt)
    x_p = zeros(8,1);
    x_p(1) = x(1,:) + x(3,:)*cosd(x(4,:))*Dt;
    x_p(2) = x(2,:) + x(3,:)*sind(x(4,:))*Dt;
    x_p(3) = x(3,:);
    x_p(4) = x(4,:) + (u(1,:)+x(5,:))*Dt;
    x_p(5) = x(5,:);
    x_p(6) = x(6,:);
    x_p(7) = x(7,:);
    x_p(8) = x(8,:);
end