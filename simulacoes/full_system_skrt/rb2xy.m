function xy = rb2xy(rb)
    xy =zeros(2,1);
    xy(1) = rb(1)*cos(rb(2));
    xy(2) = rb(1)*sin(rb(2));
end