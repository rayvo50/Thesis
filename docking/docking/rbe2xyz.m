function xyz = rbe2xyz(rbe)
    xyz = zeros(3, 1);
    xyz(1) = rbe(1) * cos(rbe(3)) * cos(rbe(2));
    xyz(2) = rbe(1) * cos(rbe(3)) * sin(rbe(2));
    xyz(3) = rbe(1) * sin(rbe(3));
end
