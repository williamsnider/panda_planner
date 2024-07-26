function [best_q] = ik_avoid_sc(panda_sc_orig, ik_orig, env, T_inter, nearby_q, num_attempts)
%IK_AVOID_SC Summary of this function goes here
%   Detailed explanation goes here

candidate_q = [];
for n = 1:num_attempts
initial_guess = randomConfiguration(panda_sc_orig);

    [q_inter, solnInfo] = ik_orig('panda_hand_tcp', T_inter, [1 1 1 1 1 1], initial_guess);
    
    
    % Check successful ik
    if ~strcmp(solnInfo.Status,"success")
        continue
    end

    % Check not in sc
    if is_robot_in_self_collision_ignore_pairs(panda_sc_orig, q_inter)
        continue
    end
    
    % Check no ec
    collisions = checkCollision(panda_sc_orig, q_inter, env);
    env_collision = collisions(2);
    if env_collision
        continue
    end

    candidate_q = [candidate_q;q_inter];

end


% Choose candidate_q closest to nearby_q
[mindist, idx] = min(sum(abs(candidate_q - repmat(nearby_q,size(candidate_q,1),1)), 2));

best_q = candidate_q(idx,:);
