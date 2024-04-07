function y = h(x)
    y = zeros(6,1);
    y(1) = x(1);
    y(2) = x(2);
    y(3) = x(3);
    y(4) = x(4);
    y(5) = sqrt((x(1)-x(6))^2 + (x(2)-x(7))^2);
    y(6) = atan2(x(2)-x(7),x(1)-x(6)) - x(4);
    y(7) = atan2(x(7)-x(2),x(6)-x(1)) - x(8);
end