function R = Rot(yaw)
    % 2D rotation maxtrix
    R = [cos(yaw), -sin(yaw);
         sin(yaw),  cos(yaw)];
end