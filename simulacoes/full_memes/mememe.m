ref = 90;
yaw = 0;
yaw_prev = 0;
pid = yaw_PID_controller();
t = 0:0.01:10;
x = zeros(6,length(t));

for 
    pid.compute(yaw)