function rb = xy2rb(xy)
    rb =zeros(2,1);
    rb(1) = sqrt(xy(1)^2+xy(2)^2);
    rb(2) = atan2d(xy(2), xy(1));
end