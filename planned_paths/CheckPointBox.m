function isInside = CheckPointBox(points, width, height, length, center)
    % Adjust half-dimensions based on the box parameters
    halfWidth = width / 2;
    halfHeight = height / 2;
    halfLength = length / 2;
    
    % Compute the limits of the box along each axis
    xMin = center(1) - halfWidth;
    xMax = center(1) + halfWidth;
    yMin = center(2) - halfLength;
    yMax = center(2) + halfLength;
    zMin = center(3) - halfHeight;
    zMax = center(3) + halfHeight;
    
    % Check if each point is within the defined limits
    isInside = points(:,1) >= xMin & points(:,1) <= xMax & ...
               points(:,2) >= yMin & points(:,2) <= yMax & ...
               points(:,3) >= zMin & points(:,3) <= zMax;
end
