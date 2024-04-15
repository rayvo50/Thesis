PB = [0,0,0];
PD = [30,20,-60]; % Dock [x,y,yaw]

P_DB = [PD(1)-PB(1);PD(2)-PB(2)];
US_B = Rot(-PB(3))*P_DB;
US_D = -Rot(-PD(3))*P_DB;

r1 = -dot(US_D,US_B);
r2 = [0,0,1]*cross([US_D;0],[US_B;0]);
atan2d(r2,r1) + PB(0);

function R = Rot(yaw)
    R = [cosd(yaw), -sind(yaw);
         sind(yaw),  cosd(yaw)];
end