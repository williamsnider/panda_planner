function valid = custom_check_valid_state(robot_ec, robot_sc, env, q, jointLimits)
    
    % Check that waypoint respects joint limits / environment&self collisions
    % This function is an alternate to using a manipulatorStateValidator
    % (e.g. isStateValid(sv,q)). The advantage is that it ignores
    % collisions between certain pairs of joints which are sometimes in
    % collision due to the collision bodies of the URDF. The core problem
    % is that some of the collision bodies in the URDF are assigned to the wrong link,
    % and so matlab will not ignore these collisions because they are not
    % adjacent or parent/child. 

    valid = true;
    
    %% Check joint limits
    if any((q-jointLimits(1,:))<0) || any((q-jointLimits(2,:))>0)
        valid = false;
        return
    end


    %% Check environment collision (ignoring certain ones)
    [inCollision, ~] = checkCollision(robot_ec, q, env, "SkippedSelfCollisions","parent");
    if inCollision
        valid = false;
        return
    end

    %% Check self collision (ignoring certain ones)
    in_self_collison = is_robot_in_self_collision_ignore_pairs(robot_sc, q);


    if in_self_collison==true
        valid = false;
    end
    
end

