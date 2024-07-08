function drawArrow(center, size, rotation, color)
    % center: 2-element vector [x, y] specifying the center of the triangle
    % size: size of the triangle
    % rotation: rotation angle in degrees (0 degrees corresponds to the beak facing up)
    % color: color of the triangle ('black', 'yellow', or 'red')
    
    % Vertices of the triangle (equilateral)
    vertices = size * [0, 4; 
                        1, 3; 
                        0.25,3;
                        0.25,-4;
                        -0.25, -4;
                        -0.25, 3
                        -1,3];
    
    % Rotation matrix
    rotationMatrix = [cosd(rotation), -sind(rotation); sind(rotation), cosd(rotation)];
    
    % Rotate the vertices
    rotatedVertices = vertices * rotationMatrix;
    
    % Translate to the specified center
    translatedVertices = rotatedVertices + center;
    
    % Plot the filled triangle with the specified color
    fill(translatedVertices(:, 1), translatedVertices(:, 2), color);
    % switch lower(color)
    %     case 'black'
    %         fill(translatedVertices(:, 1), translatedVertices(:, 2), 'k'); % black
    %     case 'yellow'
    %         fill(translatedVertices(:, 1), translatedVertices(:, 2), 'y'); % yellow
    %     case 'red'
    %         fill(translatedVertices(:, 1), translatedVertices(:, 2), 'r'); % red
    %     otherwise
    %         error('Invalid color. Use ''black'', ''yellow'', or ''red''.');
    % end
end

