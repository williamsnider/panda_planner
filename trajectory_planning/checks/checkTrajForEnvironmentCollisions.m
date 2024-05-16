function has_environment_collisions = checkTrajForEnvironmentCollisions(sv,traj, params)
    traj_steps = size(traj, 1);
    traj_gap = params.checkSteps;
    has_environment_collisions = false;

    % Shuffle timesteps to fail faster if there is a collision
    timestep_shuffled =  [1:traj_gap:traj_steps, traj_steps]; % Start + check steps + end
    timestep_shuffled = timestep_shuffled(randperm(numel(timestep_shuffled)));


    for i = 1:numel(timestep_shuffled) 
        timestep = timestep_shuffled(i);

        if ~sv.isStateValid(traj(timestep,:))
%             show(panda_sc, traj(timestep,:)); hold on;
%             plotJointMotion(panda_sc, traj(timestep,:), env, params);
%             disp(timestep)
            has_environment_collisions = true;
            break
        end
    end
end

