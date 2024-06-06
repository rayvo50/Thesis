function y = measure(x, Pd,Vc)
    position_noise = 0.3;
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
    distance = sqrt((x(1)-Pd(1))^2+(x(2)-Pd(2))^2);
    P_DB = [Pd(1)-x(1);Pd(2)-x(2)];
    USBL_B = Rot(-x(3))*P_DB + position_noise*(1+tanh(0.4*distance-3))*randn(size(P_DB));
    USBL_D = -Rot(-Pd(3))*P_DB + position_noise*(1+tanh(0.4*distance-3))*randn(size(P_DB));
    % convert to range-bearing
    y(4:5) = xy2rb(USBL_B);
    y(6:7) = xy2rb(USBL_D);
    % y(4:5) = USBL_B;
    % y(6:7) = USBL_D;

    % rate gyro
    y(8) = x(6);
    % dvl velocities in body frame
    Vc_ = Rot(-x(3))*Vc;
    y(9) =  x(4) + Vc_(1);
    y(10) = x(5) + Vc_(2);
    
    
end