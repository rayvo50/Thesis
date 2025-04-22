function R = Rot3d(yaw)
    c = cos(yaw);
    s = sin(yaw);
    R = [
        c, -s, 0;
        s, c, 0;
        0, 0, 1
    ];
end