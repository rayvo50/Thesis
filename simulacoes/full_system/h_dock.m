function y = h_dock(x_nav, x_dock)
    y = zeros(3,1);
    y(1) = sqrt((x_nav(1)-x_dock(1))^2 + (x_nav(2)-x_dock(2))^2);
    y(2) = atan2d(x_dock(2)-x_nav(2), x_dock(1)-x_nav(1)) - x_nav(4);
    y(2) = wrapTo180(y(2));
    y(3) = atan2d(x_nav(2)-x_dock(2), x_nav(1)-x_dock(1)) - x_dock(3);
    y(3) = wrapTo180(y(3));
end