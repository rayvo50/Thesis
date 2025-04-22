function y = sigmoidmoid(x)
    a=0.23;
    b=19.3;
    y = 1/(1+exp(b*(-x-a))) + 1/(1+exp(b*(-x+a))) -1;
    % if x>1
    %     y=1;
    % elseif x<-1
    %     y=-1;
    % else
    %     y=x;
    % end
end                   