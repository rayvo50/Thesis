function [u, stage_] = los_controller(x, stage,k)
    K_delta = 10;
    Ke=-1;

    alpha = 20;
    stage_=stage;
    home_D = 50;
    R = 20;
    atan2d(x(2)-x(7), x(1)-x(6));

    % check if in "entering zone"
    if abs(atan2d(x(2)-x(7), x(1)-x(6)) - x(8))<=alpha
        stage=2;
        stage_=2;
    end
 
    % homing
    if stage == 1
        x_home = x(6) + home_D*cosd(x(8));
        y_home = x(7) + home_D*sind(x(8));
        yaw_ = atan2d(y_home-x(2),x_home-x(1));
        yaw_corr = 

        if abs( (y_home-x(2))*x(6) - (x_home-x(1))*x(7) + x_home*x(2) - y_home*x(1) )/( sqrt((y_home-x(2))^2 + (x_home-x(1))^2)) < R
            R/sqrt((x(1)-x(6))^2 + (x(2)-x(7))^2)
            gamma = asind(R/sqrt((x(1)-x(6))^2 + (x(2)-x(7))^2));
            beta = atan2d(x(7)-x(2),x(6)-x(1));
            yaw_des = wrapTo180(beta-gamma);
        else
            yaw_des = atan2d(y_home-x(2),x_home-x(1))
        end
        u= [0.3;yaw_des];

    % dock path following
    elseif stage ==2
        m1 = tand(x(8));
        b1 = x(7) - m1*x(6);
       
        m2 = tand(x(8)+90);
        b2 = x(2) - m2*x(1);
       
        x_ = (b2-b1)/(m1-m2);
        y_ = m1*x_ +b1;
        
        cross_track = (m1*x(1)-x(2)+b1)/sqrt(m1^2+1);
        Delta = 2;%min(K_delta*exp(-cross_track), 5);
        x_los = x_ + Delta*cosd(x(8)+180);
        y_los = y_ + Delta*sind(x(8)+180);
        
        yaw_des = atan2d(y_los-x(2),x_los-x(1));% + Ke*cross_track;
    
        u= real([0.3;yaw_des]);
    end
end