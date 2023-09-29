function valid = motionCheck(panda_ec, panda_sc, env, paths_struct, params)
%MOTIONCHECK Checks that motions respect the kinematic limits and
%environment of the robot

% motions_ec is a cell containined joint trajectories for each motion that
% should be checked for environmental collision
% combined is an Nx9 array for the combined trajectories which should be
% checked for self collision

valid = true;

%% Prune out waypoints
paths = fieldnames(paths_struct);

% Exclude waypoints
all_paths = {};
for path_num = 1:numel(paths)
    path_name = paths{path_num};
    if contains(path_name, "wpts")
        continue
    else
        all_paths{end+1} = path_name;
    end
end

%% Construct list of paths

ec_exclusion = ["out_to_above", "above_to_slot", "slot_to_above", "above_to_out"];
ec_paths = cell(0);
for path_num =1:numel(all_paths)
    path_name = all_paths{path_num};

    excluded_path = false;
    for ec_exclusion_num = 1:numel(ec_exclusion)
        exclusion_name = ec_exclusion(ec_exclusion_num);

        % Exclude cartesian paths
        if contains(path_name, exclusion_name)
            excluded_path = true;
            break
        end



    end

    if excluded_path==false
        ec_paths{end+1} = path_name;
    end
end



%% Check joint kinematics (position, velocity, acceleration, jerk)
for path_num = 1:numel(all_paths)
    traj = getfield(paths_struct, all_paths{path_num});
    if ~checkTrajectory(traj, traj(1,:), traj(end,:), params)
        valid = false;
        disp("invalid kinematics for path: "+all_paths{path_num})
    end
end


%% Check environmental collisions (for non-cartesian paths)
ss = ManipulatorStateSpaceSphere(panda_ec, panda_sc);
radius_offset = 0;  % Do not offset the radius at all (makes it more generous)
sv = ManipulatorStateValidatorSphere(ss, env, params.validationDistance, radius_offset, params);
sv.IgnoreSelfCollision = false;
sv.Environment = env;

for path_num =1:numel(ec_paths)

    collision_count = 0;

    path_name = ec_paths{path_num};
%     disp(path_name)
    traj = getfield(paths_struct, path_name);
    traj_steps = size(traj, 1);
    traj_gap = params.checkSteps;
    for timestep = [1:traj_gap:traj_steps, traj_steps] % Start + check steps + end
        if ~sv.isStateValid(traj(timestep,:))
%             show(panda_sc, traj(timestep,:)); hold on;
%             plotJointMotion(panda_sc, traj(timestep,:), env, params);
%             disp(timestep)
            collision_count = collision_count + 1;
        end
    end

    if collision_count > 1
                    valid = false;
        disp("motionCheck - env collision with number of instances: "+num2str(collision_count))
        return
    end
end

%% Check self collisions for all positions
for path_num = 1:numel(all_paths)
    path_name = all_paths{path_num};
    traj = getfield(paths_struct, path_name);
    traj_steps = size(traj,1);
    traj_gap = params.checkSteps;
    for timestep = [1:traj_gap:traj_steps, traj_steps] % Start + check steps + end
        if is_robot_in_self_collision_ignore_pairs(panda_sc, traj(timestep,:))
            valid=false;
            disp("motionCheck - self collision")
            return
        end
    end

end

end

