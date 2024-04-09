function x_p = f_nav(x,u,Dt)
    x_p = zeros(5,1);
    x_p(1) = x(1,:) + x(3,:)*cosd(x(4,:))*Dt;
    x_p(2) = x(2,:) + x(3,:)*sind(x(4,:))*Dt;
    x_p(3) = x(3,:);
    x_p(4) = x(4,:) + (u(1,:)+x(5,:))*Dt;
    x_p(5) = x(5,:);
end