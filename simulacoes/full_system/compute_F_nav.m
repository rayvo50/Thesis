function F = compute_F_nav(x, Dt)
    F = eye(5);
    F(1,3) = cosd(x(4));
    F(2,3) = sind(x(4));
    F(1,4) = -x(3)*sind(x(4));
    F(2,4) = x(3)*cosd(x(4));
    F(4,5) = 1;
    sys_d = c2d(ss(F,[0,0,0,1,0]',eye(5),0),Dt,'foh');
    F = sys_d.A;
end