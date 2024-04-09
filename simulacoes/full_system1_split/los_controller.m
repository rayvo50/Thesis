function u = los_controller(x)
    Delta_max =10;
    
    % initial point to aim to:
    [cosd(x(8)), sind(x(8))];

    m1 = tand(x(8));
    b1 = x(7) - m1*x(6);
   
    m2 = tand(x(8)+90);
    b2 = x(2) - m2*x(1);
   
    x_ = (b2-b1)/(m1-m2);
    y_ = m1*x_ +b1;
    
    cross_track = abs(m1*x(1)-X(2)+b1)/sqrt(m1^2+1);
    Delta = Delta_max*exp(-cross_track);
    x_los = x_ + Delta*cosd(x(8)+180);
    y_los = y_ + Delta*sind(x(8)+180);
    
    yaw_des = atan2d(y_los-x(2),x_los-x(1));

    u= [0.3;yaw_des];
end