
function [dist_from_edge_arr, q_list_arr] = find_best_q_for_cartesian_path(T0, TRAVEL_DIST, panda_sc,ik, params)

TF = T0;
TF(1,4) = TF(1,4)+TRAVEL_DIST;
nSamples = 5;
tSamples = linspace(0,1,nSamples);
tInterval = [0,1];
[tforms,~,~] = transformtraj(T0,TF,tInterval,tSamples);

num_loops = 200;
dist_from_edge_arr = zeros(1,num_loops);
q_list_arr = cell(1, num_loops);
for loop_count = 1:num_loops
    initialGuess = randomConfiguration(panda_sc);
    [q_initial,solnInfo] = ik('panda_hand_tcp',tforms(:,:,1),[1 1 1 1 1 1],initialGuess);
    if strcmp(solnInfo.Status, "best available")
        continue
    end

    % Check that there are no self collisions for it or rotations of J7
    J7_list = (-pi:pi/2:pi/2) + q_initial(7);
    J7_list(J7_list > 2.89) = J7_list(J7_list > 2.89) - 2*pi;
    J7_list(J7_list < -2.89) = J7_list(J7_list < -2.89) + 2*pi;
    J7_list_valid = true;
    for J7_num = 1:4
        q_to_test = q_initial;
        q_to_test(7) = J7_list(J7_num);
        if is_robot_in_self_collision_ignore_pairs(panda_sc,q_to_test)
            J7_list_valid=false;
            break
        end
    end
    if J7_list_valid==false
        continue
    end



    q_list = [q_initial];
    scaled_list = calc_scaled(q_initial, params);
    for sample_num = 2:nSamples
        [q,solnInfo] = ik('panda_hand_tcp',tforms(:,:,sample_num),[1,1,1,1,1,1],q_list(sample_num-1,:));
        if strcmp(solnInfo.Status, "best available")
            dummy_var = 0;
        end
        q_list = [q_list;q];
        scaled_list = [scaled_list; calc_scaled(q, params)];
    end
    dist_from_upper = 1-max(max(scaled_list));
    dist_from_lower = min(min(scaled_list));
    dist_from_edge = min(dist_from_upper, dist_from_lower);

    % Confirm jVals not to close to min/max
    j7 = q_list(1,7);
    jVals = j7:pi/2:j7+3*pi/2;
    jVals(jVals > params.jointMax(7)) = jVals(jVals >params.jointMax(7)) - 2*pi;
    if any(abs(jVals-params.jointMax(7))< 0.1)
        dist_from_edge=0;
    elseif any(any(abs(jVals-params.jointMin(7))< 0.1))
        dist_from_edge=0;
    end

    j7 = q_list(end,7);
    jVals = j7:pi/2:j7+3*pi/2;
    jVals(jVals > params.jointMax(7)) = jVals(jVals >params.jointMax(7)) - 2*pi;
    if any(abs(jVals-params.jointMax(7))< 0.3)
        dist_from_edge=0;
    elseif any(any(abs(jVals-params.jointMin(7))< 0.3))
        dist_from_edge=0;
    end

    % Confirm robot not in self collision

    dist_from_edge_arr(loop_count) = dist_from_edge;
    q_list_arr{loop_count} = q_list;

end

end