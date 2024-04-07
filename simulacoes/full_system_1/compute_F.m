function F = compute_F(x, Dt)
    F = eye(8);
    F(1,3) = cosd(x(4,:))*Dt;
    F(2,3) = sind(x(4,:))*Dt;
    F(1,4) = -x(3,:)*sind(x(4,:))*Dt;
    F(2,4) = x(3,:)*cosd(x(4,:))*Dt;
    F(4,5) = Dt;
end