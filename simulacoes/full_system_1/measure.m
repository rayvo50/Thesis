function y = measure(x, Pd)
    position_noise = 0.5;
    compass_noise = 5;
    range_noise = 0.5;
    bearing_noise = 2;
    dvl_noise = 0.005;
    y = zeros(7,1);
    y(1) = x(1) + position_noise*randn(size(x(1)));
    y(2) = x(2) + position_noise*randn(size(x(1)));
    y(3) = x(3) + dvl_noise*randn(size(x(1)));
    y(4) = x(4) + compass_noise*randn(size(x(1)));
    y(5) = sqrt((x(1)-Pd(1))^2 + (x(2)-Pd(2))^2) + range_noise*randn(size(x(1)));
    if y(5) < 60
        y(6) = rad2deg(atan2(x(2)-Pd(2),x(1)-Pd(1))) - x(4) + bearing_noise*randn(size(x(1)));
        if y(4) < 30
            y(7) = rad2deg(atan2(Pd(2)-x(2),Pd(1)-x(1))) - Pd(3) + bearing_noise*randn(size(x(1)));
        end
    end
end