function y = measure(x, Pd)
    position_noise = 0;
    compass_noise = 0;
    range_noise = 0;
    bearing_noise = 0;
    dvl_noise = 0;
    gyro_bias = 0;
    y = zeros(9,1);
    % position from surface vessel
    y(1) = x(1) + position_noise*randn(size(x(1)));
    y(2) = x(2) + position_noise*randn(size(x(1)));
    % yaw from AHRS
    y(3) = x(3) + compass_noise*randn(size(x(1)));
    % dvl velocities
    y(4) = x(4) + dvl_noise*randn(size(x(1)));
    y(5) = x(5) + dvl_noise*randn(size(x(1)));
    % USBL measurements
    P_DB = [Pd(1)-x(1);Pd(2)-x(2)];
    USBL_B = Rot(x(3))*P_DB;
    USBL_D = -Rot(Pd(3))*P_DB;
    y(6) = USBL_B(1);
    y(7) = USBL_B(2); 
    y(8) = USBL_D(1);
    y(9) = USBL_D(2);
    % y(6) = sqrt(USBL_B(1)^2+USBL_B(2)^2);
    % y(7) = atan2d(USBL_B(2),USBL_B(1));
    % y(8) = sqrt(USBL_D(1)^2+USBL_D(2)^2);
    % y(9) = atan2d(USBL_D(2),USBL_D(1));

    % y(5) = sqrt((x(1)-Pd(1))^2 + (x(2)-Pd(2))^2) + range_noise*randn(size(x(1)));
    % if y(5) < 20000
    %     y(6) = atan2d(Pd(2)-x(2),Pd(1)-x(1)) - x(4) + bearing_noise*randn(size(x(1)));
    %     y(6) = wrapTo180(y(6));
    %     if y(4) < 20000
    %         y(7) = atan2d(x(2)-Pd(2),x(1)-Pd(1)) - Pd(3) + bearing_noise*randn(size(x(1)));
    %         y(7) = wrapTo180(y(7));
    %     end
    % end
    % y(8) = x(5) + gyro_bias;
end