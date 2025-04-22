function y = measure(x, Pd,Vc)
    position_noise = 0.3;
    compass_noise = deg2rad(4);
    dvl_noise = 0.01;
    gyro_bias = 0;
    y = zeros(14,1);

    % position from surface vessel
    y(1) = x(1) + position_noise*randn();
    y(2) = x(2) + position_noise*randn();
    y(3) = x(3) + position_noise*randn();
    % yaw from AHRS
    y(4) = x(4) + compass_noise*randn();

    % USBL measurements
    P_DB = Pd(1:3)-x(1:3);
    USBL_B = Rot3d(-x(4))*P_DB;
    USBL_D = -Rot3d(-Pd(4))*P_DB;
    % convert to range-bearing
    y(5:7) = diag([1+0.05*randn,1+0.05*randn,1+0.05*randn])*xyz2rbe(USBL_B);
    y(8:10) = diag([1+0.05*randn,1+0.05*randn,1+0.05*randn])*xyz2rbe(USBL_D);

    % dvl velocities in body frame
    y(11:13) =  x(5:7) +  Rot3d(-x(4))*Vc + dvl_noise*randn(3,1);
    % rate gyro
    y(14) = x(8);
end