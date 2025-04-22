function y = measure(X, terrain, alpha)
    % Computes the measurements from both echosounders, given the vehicle's
    % pose, a funtion decribing the terrain profile and the installation angle
    % of the second echo sounder.
    % The function does a simple raytracing using fzero to find the
    % intersection of the beam with the terrain
    
    % This is the downwards echosounder. TODO: take pitch into acount
    h1 = terrain(X(1))-X(2);

    
    ray2 = @(x) tan(pi/2-alpha) * x + X(2) - tan(pi/2-alpha)*X(1);  % 2nd echosounder ray function
    expr = @(x) terrain(x) - ray2(x);   % expression to find zero
    intersection = fzero(expr, X(1));   

    h2= norm([X(1) - intersection, X(2)- terrain(intersection)]); % compute range to the computed intersection point
    
    y=[X(3),X(4),h1*(1+0.01*randn), h2*(1+0.01*randn)]; % Add gaussian noise

    % Add some outliers
    if rand <0.05
        y(3) = y(3) + sign(rand-0.5)*(rand*15+5);
        y(4) = y(4) + sign(rand-0.5)*(rand*15+5);
    end

    % uncomment below to get perfect measurements
    % y=[X(3),X(4),h1, h2];
    
end