function R = rodrigues_rotation(v, theta)
    % Convert theta from degrees to radians
    theta = deg2rad(theta);

    % Normalize the vector
    v = v / norm(v);

    % Compute the skew-symmetric matrix
    vx = [0, -v(3), v(2);
          v(3), 0, -v(1);
          -v(2), v(1), 0];

    % Rodriguez rotation formula
    R = eye(3) + sin(theta) * vx + (1 - cos(theta)) * vx^2;
end