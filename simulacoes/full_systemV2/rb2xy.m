function xy = rb2xy(rb)
    xy =zeros(2,1);
    xy(1) = rb(1)*cosd(rb(2));
    xy(2) = rb(1)*sind(rb(2));
end