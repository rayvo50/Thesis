function output = sigma_e(input)
    if (input > 1)
        output =  1;
    elseif (input < -1)
        output = -1;
    else
        output = input;
    end
end
