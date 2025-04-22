function rbe = xyz2rbe(xyz)
    rbe = zeros(3, 1);
    rbe(1) = norm(xyz);                  
    rbe(2) = atan2(xyz(2), xyz(1));      
    rbe(3) = atan2(xyz(3), sqrt(xyz(1)^2 + xyz(2)^2)); 
end
