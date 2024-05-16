function has_self_collisions = checkTrajForSelfCollisions(panda_sc,traj, params)
%CHECKTRAJSELFCOLLISION Summary of this function goes here
    traj_steps = size(traj,1);
    traj_gap = params.checkSteps;
    
    has_self_collisions = false;
    for timestep = [1:traj_gap:traj_steps, traj_steps] % Start + check steps + end
        if is_robot_in_self_collision_ignore_pairs(panda_sc, traj(timestep,:))
            has_self_collisions=true;
            break
        end
    end
end

