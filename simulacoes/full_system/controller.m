function u = controller(x,Pd)
    u = zeros(2,1); 
    %if sqrt((x(1)-Pd(1))^2 + (x(2)-Pd(2))^2)
    u(1) = 0.3;
    u(2) = atan2d(Pd(2)-x(2), Pd(1)-x(1));
    %end
end