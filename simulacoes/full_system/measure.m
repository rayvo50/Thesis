function y = measure(x, Pd)
    position_noise = 0.5;
    compass_noise = deg2rad(1);
    dvl_noise = 0.001;
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
    x1 = USBL_B(1)+position_noise*randn();
    y1 = USBL_B(2)+position_noise*randn(); 
    x2 = USBL_D(1)+position_noise*randn();
    y2 = USBL_D(2)+position_noise*randn();

    % y(4:5) = [x1;y1];
    % y(6:7) = [x2;y2];
    % if sqrt((x(1)-Pd(1))^2 + (x(2)-Pd(2))^2) < 40
    %     y(4) = sqrt(x1^2+y1^2);
    %     y(5:7) = [NaN;NaN;NaN];
    % else
    y(4:5) = xy2rb([x1;y1]);
    y(6:7) = xy2rb([x2;y2]);
    % end

    % rate gyro
    y(8) = x(6);
    % dvl velocities
    y(9) = x(4);
    y(10) = x(5);
    
    
end