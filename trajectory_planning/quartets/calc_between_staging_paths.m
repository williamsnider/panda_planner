function [cell_staging_to_inter_70, cell_staging_to_inter_path, cell_inter_to_inter_70, cell_inter_to_inter_path] = calc_between_staging_paths(arr,inter_shift,panda_ec_orig, panda_sc_orig,ik_orig, env, params)
%CALC_BETWEEN_STAGING_PATHS Summary of this function goes here


inter = zeros(size(arr));
num_positions = size(arr,1);

%% Calculate inter positions
for i = 1:num_positions

    q = arr(i,:);
    T = getTransform(panda_sc_orig, q, 'panda_hand_tcp');
    T_inter = T + inter_shift;
    [q_inter, solnInfo] = ik_orig('panda_hand_tcp', T_inter, [1 1 1 1 1 1], q);

    % Checks
    assert(strcmp(solnInfo.Status,"success"))
    assert(~is_robot_in_self_collision_ignore_pairs(panda_sc_orig, q_inter))
    collisions = checkCollision(panda_sc_orig, q_inter, env);
    env_collision = collisions(2);
    assert(~env_collision)
    assert(sum(abs(q_inter-q))<1.0)

    inter(i,:) = q_inter;
end


% Calculate staging to inter at 70% vMax
cell_staging_to_inter_70 = {};
cell_staging_to_inter_path = {};

for i = 1:num_positions

    start = arr(i,:);
    goal = inter(i,:);

    [traj, planned_path] = planJointToJoint(panda_ec_orig, panda_sc_orig, env, start, goal, params);
    cell_staging_to_inter_70{end+1} = traj;
    cell_staging_to_inter_path{end+1} = planned_path;

    disp(size(traj))
    disp(size(planned_path))
end



% Calculating inter to inter
cell_inter_to_inter_70 = cell(num_positions, num_positions);
cell_inter_to_inter_path = cell(num_positions, num_positions);

parfor r=1:num_positions
    for c = 1:num_positions
        if c<=r
            continue
        end

        disp([r,c])
        start = inter(r,:);
        goal = inter(c,:);
        [traj, planned_path] = planJointToJoint(panda_ec_orig, panda_sc_orig, env, start, goal, params);
        %     plotJointMotion(panda_sc_orig, traj, env, params)
        cell_inter_to_inter_70{r,c} = traj;
        cell_inter_to_inter_path{r,c} = planned_path;
    end
end


%% Mirror inter_to_inter array by flipping paths and trajectories
for i = 1:num_positions
    for j = i:num_positions
        if i==j
            continue
        end

        % Insert into array
        cell_inter_to_inter_70{j,i}= flip(cell_inter_to_inter_70{i,j},1);
        cell_inter_to_inter_path{j,i}= flip(cell_inter_to_inter_path{i,j},1);
    end
end






end

