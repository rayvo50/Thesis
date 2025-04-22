function D = get_real_d(X, terrain)
    % Copmutes the real vector D at a given time for analysing the results
    % X: [x, y] - point coordinates
    % terrain: function handle defining the curve y = terrain(x)

    % Define the distance function
    distance = @(x) sqrt((x - X(1))^2 + (terrain(x) - X(2))^2);

    % Use fminbnd to find the x-coordinate of the closest point on the curve
    options = optimset('MaxIter', 100000, ... % Maximum function evaluations
                   'TolX', 1e-12 ...       % Tolerance for solution
                   );     % Display progress during optimization
    x_min = fminsearch(distance, X(1), options);
    % x_min = fminsearch(distance, X(1)); % Adjust bounds if needed

    % Compute the closest point on the curve
    closest_point = [x_min; terrain(x_min)];

    % Compute the vector D
    D = closest_point - X(1:2);
end
