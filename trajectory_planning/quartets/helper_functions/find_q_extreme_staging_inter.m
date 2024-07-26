function [q_extreme_12, q_staging_12, q_inter_12, elbow_12] = find_q_extreme_staging_inter(SV, T_extreme, body_name, T_extreme_to_staging, T_staging_to_inter, panda_sc_restricted, panda_sc_orig, ik_restricted, ik_orig, env, params)
    %FIND_Q_EXTREME_STAGING_INTER Summary of this function goes here

    extreme_candidates = [];
    staging_candidates = [];
    inter_candidates = [];

    for i = 1:100


        T_staging = T_extreme + T_extreme_to_staging;
        T_inter = T_staging + T_staging_to_inter;

        % Generate extreme
        initialGuess = randomConfiguration(panda_sc_restricted);
        initialGuess(8:9) = 0.01;
        [q_extreme, solnInfo] = find_q_from_T(panda_sc_restricted, body_name, T_extreme, ik_restricted, initialGuess);

        if ~strcmp(solnInfo.Status, "success") || is_robot_in_self_collision_ignore_pairs(panda_sc_restricted, q_extreme)
            continue
        end

        % Generate staging from extreme
        [q_staging, solnInfo] = find_q_from_T(panda_sc_orig, body_name, T_staging, ik_orig, q_extreme);

        if ~strcmp(solnInfo.Status, "success") || is_robot_in_self_collision_ignore_pairs(panda_sc_orig, q_staging)
            continue
        end

        % Check environmental collisions
        if ~SV.isStateValid(q_staging)
            continue
        end

        MAX_DIST = 1.0;
        dist = sum(abs((q_staging - q_extreme)));
        if dist > MAX_DIST
            continue
        end

        % Generate inter from staging
        [q_inter, solnInfo] = find_q_from_T(panda_sc_orig, body_name, T_inter, ik_orig, q_staging);

        if ~strcmp(solnInfo.Status, "success") || is_robot_in_self_collision_ignore_pairs(panda_sc_orig, q_inter)
            continue
        end

        % Check environmental collisions
        if ~SV.isStateValid(q_inter)
            continue
        end

        dist = sum(abs((q_staging - q_inter)));
        if dist > MAX_DIST
            continue
        end

        % If all passes, keep values
        extreme_candidates = [extreme_candidates; q_extreme];
        staging_candidates = [staging_candidates; q_staging];
        inter_candidates = [inter_candidates; q_inter];
    end

    % Breakpoint for debugging
    if numel(extreme_candidates)==0
        dummy=0;
    end


    % Choose two furthest q_extremes based on joint 1
    [~, minidx] = min(extreme_candidates(:,1));
    [~, maxidx] = max(extreme_candidates(:,1));
    q_extreme_12 = [extreme_candidates(minidx,:); extreme_candidates(maxidx,:)];

    % Recalculate q_staging using incremental steps (ensures that elbow
    % steps align exactly with q_staging)
    q_staging_12 = [staging_candidates(minidx,:); staging_candidates(maxidx,:)];
    
    % First q_extreme
    num_steps = 1001;
    T_extreme_TCP = getTransform(panda_sc_orig, q_extreme_12(1,:), 'panda_hand_tcp');
    T_staging_TCP = T_extreme_TCP+T_extreme_to_staging;
    [q_extreme_to_staging_arr, z_vals_extreme_to_staging] = calc_cartesian_path_q1_q2(q_extreme_12(1,:), q_staging_12(1,:), T_extreme_TCP, T_staging_TCP, ik_orig, panda_sc_orig, 'panda_hand_tcp', num_steps);
    elbow_LUT_1 = construct_elbow_LUT(q_extreme_to_staging_arr,z_vals_extreme_to_staging);
    assert(sum(sum(abs(elbow_LUT_1(1,:)-q_staging_12(1,1:7))))<0.0001);

    % Second q_extreme
    T_extreme_TCP = getTransform(panda_sc_orig, q_extreme_12(2,:), 'panda_hand_tcp');
    T_staging_TCP = T_extreme_TCP+T_extreme_to_staging;
    [q_extreme_to_staging_arr, z_vals_extreme_to_staging] = calc_cartesian_path_q1_q2(q_extreme_12(2,:), q_staging_12(2,:), T_extreme_TCP, T_staging_TCP, ik_orig, panda_sc_orig, 'panda_hand_tcp', num_steps);
    elbow_LUT_2 = construct_elbow_LUT(q_extreme_to_staging_arr,z_vals_extreme_to_staging);
    assert(sum(sum(abs(elbow_LUT_2(1,:)-q_staging_12(2,1:7))))<0.0001);
    
    elbow_12 = {elbow_LUT_1, elbow_LUT_2};

    q_inter_12 = [inter_candidates(minidx,:); inter_candidates(maxidx,:)];
end
