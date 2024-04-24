function y = measure(x, Pd)
    position_noise = 0.01;
    compass_noise = 1;
    dvl_noise = 0.01;
    gyro_bias = 0;
    y = zeros(9,1);

    % position from surface vessel
    y(1) = x(1) + position_noise*randn();
    y(2) = x(2) + position_noise*randn();
    % yaw from AHRS
    y(3) = x(3) + compass_noise*randn();
    
    % USBL measurements
    P_DB = [Pd(1)-x(1);Pd(2)-x(2)];
    USBL_B = Rot(-x(3))*P_DB;
    USBL_D = -Rot(-Pd(3))*P_DB;
    y(4) = USBL_B(1)+position_noise*randn();
    y(5) = USBL_B(2)+position_noise*randn(); 
    y(6) = USBL_D(1)+position_noise*randn();
    y(7) = USBL_D(2)+position_noise*randn();
    % y(6) = sqrt(USBL_B(1)^2+USBL_B(2)^2);
    % y(7) = atan2d(USBL_B(2),USBL_B(1));
    % y(8) = sqrt(USBL_D(1)^2+USBL_D(2)^2);
    % y(9) = atan2d(USBL_D(2),USBL_D(1));

    % dvl velocities
    y(8) = x(4) + dvl_noise*randn();
    %y(9) = x(4)*sind(x(3)) + dvl_noise*randn();

end