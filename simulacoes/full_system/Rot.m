function R = Rot(yaw)
        R = [cosd(yaw), -sind(yaw);
             sind(yaw),  cosd(yaw)];
end