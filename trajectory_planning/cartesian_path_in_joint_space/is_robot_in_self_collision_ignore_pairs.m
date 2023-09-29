function in_self_collision = is_robot_in_self_collision_ignore_pairs(robot_sc, q)
%% Checks self collision but ignores pairs of bodies that are mistakenly colliding due to the URDF

    [~, sepDist] = checkCollision(robot_sc, q,{}, "SkippedSelfCollisions","parent");

    % Ignore pairs of links that are sometimes in collision due to the URDF
    ignore_pairs = [3,5;5,7];  % Bodies 3 and 5 are mistakenly in self collision sometimes.
    for i=1:size(ignore_pairs,1)
        idx1 = ignore_pairs(i,1);
        idx2 = ignore_pairs(i,2);
        sepDist(idx1,idx2) = 0;
        sepDist(idx2,idx1) = 0;
    end

    if any(any(isnan(sepDist)))
        in_self_collision = true;
    else
        in_self_collision= false;
    end
end