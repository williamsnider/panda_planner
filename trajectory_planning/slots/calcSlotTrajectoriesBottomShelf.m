function paths_struct= calcSlotTrajectories(panda_ec, panda_sc, env, ik, q_slot, stateBounds,staging_data, params)

%% Gather needed parameters
ABOVE_HEIGHT = params.ABOVE_HEIGHT;
% vMaxAll = params.vMaxAll;
% aMaxAll = params.aMaxAll;
% jMaxAll = params.jMaxAll;
% vMaxAllAbsolute = params.vMaxAllAbsolute;
% aMaxAllAbsolute = params.aMaxAllAbsolute;
% jMaxAllAbsolute = params.jMaxAllAbsolute;
% jointMin = params.jointMin;
% jointMax = params.jointMax;
q_home = params.q_home;

ss = ManipulatorStateSpaceEllipsoid(panda_ec, panda_sc);
sv = ManipulatorStateValidatorEllipsoid(ss, env, params.validationDistance,params.ellipsoid_radius_offset, params);
sv.IgnoreSelfCollision = false;
sv.Environment = env;



%% Calculate poses for pick/place
T_slot = getTransform(panda_sc, q_slot, "panda_hand_tcp");

T_downIn = T_slot;
T_upIn = T_downIn;
T_upIn(3,4) = T_upIn(3,4) + ABOVE_HEIGHT;


% T_above = T_slot;
% T_above(3,4) = T_slot(3,4) + ABOVE_HEIGHT;

% Calculate minimum OUT_DIST to pull shape within sphere
for OUT_DIST = 0.0:0.005:0.4

    % Calculate T_upOut
    T_upOut = T_upIn;
    T_upOut(1:3,4) = T_upOut(1:3,4) - T_upOut(1:3,3)*OUT_DIST;

    % Calculate T_aboveOut (second shelf)
    T_aboveOut = T_upOut;
    T_aboveOut(3,4) = T_aboveOut(3,4) + params.Z_list(2) - params.Z_list(1);  % Spacing between shelves

%     % Calculate T_out
%     T_out = T_slot;
%     T_out(1:3,4) = T_out(1:3,4) - T_out(1:3,3)*OUT_DIST;

%     % Do inverse kinematics
%     initialGuess = q_slot;
%     weights = [1 1 1 1 1 1];
%     [q_upOut,solnInfo] = ik('panda_hand_tcp',T_upOut,weights,initialGuess);
% 
%     % Break loop if valid (inside ellipsoidf for both T_upOut and T_downOut)
%     if sv.isStateValid(q_upOut)
% 
%         % Test if q_downOut valid too
%         T_downOut = T_upOut;
%         T_downOut(3,4)= T_downIn(3,4);
%         [q_downOut,solnInfo] = ik('panda_hand_tcp',T_downOut,weights,q_upOut);
%         if sv.isStateValid(q_downOut)
%             break
%         end
%     end

    % Do inverse kinematics
    initialGuess = q_slot;
    weights = [1 1 1 1 1 1];
    [q_aboveOut,solnInfo] = ik('panda_hand_tcp',T_aboveOut,weights,initialGuess);

    % Break loop if valid (inside ellipsoid for q_aboveOut)
    if sv.isStateValid(q_aboveOut)

        % Calculate q_upOut and q_downOut (both probably outside of
        % ellipsoid)
        initialGuess = q_slot;
        weights = [1 1 1 1 1 1];
        [q_upOut,solnInfo] = ik('panda_hand_tcp',T_upOut,weights,initialGuess);
        T_downOut = T_upOut;
        T_downOut(3,4)= T_downIn(3,4);
        [q_downOut,solnInfo] = ik('panda_hand_tcp',T_downOut,weights,q_upOut);
        break
    end




    if  OUT_DIST == 0.2
%         plotJointMotion(panda_sc, q_aboveOut, env, params)
%         disp('here')
    end
end
% plotJointMotion(panda_sc, q_aboveOut, env, params)
% disp('here')
assert(sv.isStateValid(q_aboveOut))


% T_downIn = T_slot;
T_downOut = T_upOut;
T_downOut(3,4)= T_downIn(3,4);


% Check Z's 
assert(abs(T_downOut(3,4)-T_downIn(3,4))<0.015)
assert(abs(T_upOut(3,4)-T_upIn(3,4))<0.015)


T_downIn_to_downOut = cat(3, T_downIn, T_downOut);

% 
% show(panda_sc, q, 'Collisions','on'); hold on;
% plotJointMotion(panda_sc, q, env, params);

% 
% % Plot robot positions
% T_cell = {T_slot, T_above, T_out};
% initialGuess = q_home;
% for T_idx = 1:numel(T_cell)
% 
%     T = T_cell{T_idx};
%     [q,solnInfo] = ik('panda_custom_shape', T, weights, initialGuess);
%     assert(strcmp(solnInfo.Status, "success"))
%     initialGuess = q;
% 
%     show(panda_sc, q); hold on;
% end

%% Calculate trajectories
paths_struct = struct();

% Cartesian paths for pick/place
T_downIn_to_upIn_to_upOut_to_aboveOut = cat(3,T_downIn, T_upIn, T_upOut, T_aboveOut);
found_valid_cartesian_path = false;

% Use known q for q_slot
q_downIn = q_slot;

%Redo inverse kinematics if q is outside joint limits or in collision
q_downIn = round(q_downIn, 10);  % Rounding ensures not exceeding joint limits by precision error

%     if ~custom_check_valid_state(panda_ec, panda_sc, env, q, stateBounds)
%         continue
%     end



% For the bottom shelf, simplying pulling from upIn to upOut does not give
% enough room. So, we must add a third vertical motion to pull the EE to
% the next shelf's region (inside the ellipsoid). This vertical motion will
% be appended to each of the relevant joint motions (home_to_upOut becomes home_to_aboveOut_to_upOut). 

% Tricky - how to approach upOut to downOut? Since these are two joint
% goals. Just use normal joint motion here (with no environment collision
% checking and only large motion checking - small motion unlikely to cause problems)

vScale = 0.1;
[all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_upIn_to_upOut_to_aboveOut,q_downIn,params);
assert(all_valid)
[downIn_to_downOut_path, downIn_to_downOut_valid, downIn_to_downOut_all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_downOut,q_downIn,params);
assert(downIn_to_downOut_valid)
all_paths{4} = all_paths{3}; % move upOut_to_aboveOut to 4th position
all_paths{3} = downIn_to_downOut_path{1};  % Insert downIn_to_downOut_path to position 3;
all_wpts = [all_wpts;all_wpts(end-8:end,:)]; % move wpts_upOut_to_aboveOut to last position
all_wpts(19:27,:) = downIn_to_downOut_all_wpts;
paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);

vScale = 0.4;
[all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_upIn_to_upOut_to_aboveOut,q_downIn,params);
assert(all_valid)
[downIn_to_downOut_path, downIn_to_downOut_valid, downIn_to_downOut_all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_downOut,q_downIn,params);
assert(downIn_to_downOut_valid)
all_paths{4} = all_paths{3}; % move upOut_to_aboveOut to 4th position
all_paths{3} = downIn_to_downOut_path{1};  % Insert downIn_to_downOut_path to position 3;
all_wpts = [all_wpts;all_wpts(end-8:end,:)]; % move wpts_upOut_to_aboveOut to last position
all_wpts(19:27,:) = downIn_to_downOut_all_wpts;
paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);

vScale = 0.7;
[all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_upIn_to_upOut_to_aboveOut,q_downIn,params);
assert(all_valid)
[downIn_to_downOut_path, downIn_to_downOut_valid, downIn_to_downOut_all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_downOut,q_downIn,params);
assert(downIn_to_downOut_valid)
all_paths{4} = all_paths{3}; % move upOut_to_aboveOut to 4th position
all_paths{3} = downIn_to_downOut_path{1};  % Insert downIn_to_downOut_path to position 3;
all_wpts = [all_wpts;all_wpts(end-8:end,:)]; % move wpts_upOut_to_aboveOut to last position
all_wpts(19:27,:) = downIn_to_downOut_all_wpts;
paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);

% Calculate upOut_to_aboveOut_to_downOut




% 
% % 
% vScale = 0.1;
% [all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_upIn_to_upOut_to_aboveOut,q_downIn,params);
% assert(all_valid)
% [downIn_to_downOut_path, downIn_to_downOut_valid, downIn_to_downOut_all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_downOut,q_downIn,params);
% assert(downIn_to_downOut_valid)
% all_paths{3} = downIn_to_downOut_path{1};
% all_wpts(end+1:end+9,:) = downIn_to_downOut_all_wpts;
% paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);


% vScale = 0.4;
% [all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_upIn_to_upOut_to_aboveOut,q_downIn,params);
% assert(all_valid)
% [downIn_to_downOut_path, downIn_to_downOut_valid, downIn_to_downOut_all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_downOut,q_downIn,params);
% assert(downIn_to_downOut_valid)
% all_paths{3} = downIn_to_downOut_path{1};
% all_wpts(end+1:end+9,:) = downIn_to_downOut_all_wpts;
% paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);
% 
% vScale = 0.7;
% [all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_upIn_to_upOut_to_aboveOut,q_downIn,params);
% assert(all_valid)
% [downIn_to_downOut_path, downIn_to_downOut_valid, downIn_to_downOut_all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_downOut,q_downIn,params);
% assert(downIn_to_downOut_valid)
% all_paths{3} = downIn_to_downOut_path{1};
% all_wpts(end+1:end+9,:) = downIn_to_downOut_all_wpts;
% paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);
% 


% Ensure out joint position identical
assert(all(paths_struct.upIn_to_upOut_10(end,:)-paths_struct.upIn_to_upOut_40(end,:)<0.00000001))
assert(all(paths_struct.upIn_to_upOut_40(end,:)-paths_struct.upIn_to_upOut_70(end,:)<0.00000001))
q_upOut = paths_struct.upIn_to_upOut_10(end,:);
q_upOut = [q_upOut, 0.01, 0.01];

assert(all(paths_struct.upIn_to_upOut_10(1,:)-paths_struct.upIn_to_upOut_40(1,:)<0.00000001))
assert(all(paths_struct.upIn_to_upOut_40(1,:)-paths_struct.upIn_to_upOut_70(1,:)<0.00000001))
q_upIn = paths_struct.upIn_to_downIn_10(1,:);
q_upIn = [q_upIn, 0.01, 0.01];

assert(all(paths_struct.downIn_to_downOut_10(end,:)-paths_struct.downIn_to_downOut_40(end,:)<0.00000001))
assert(all(paths_struct.downIn_to_downOut_40(end,:)-paths_struct.downIn_to_downOut_70(end,:)<0.00000001))
q_downOut = paths_struct.downIn_to_downOut_10(end,:);
q_downOut = [q_downOut, 0.01, 0.01];

assert(all(paths_struct.upOut_to_aboveOut_10(end,:)-paths_struct.upOut_to_aboveOut_40(end,:)<0.00000001))
assert(all(paths_struct.upOut_to_aboveOut_40(end,:)-paths_struct.upOut_to_aboveOut_70(end,:)<0.00000001))
q_aboveOut = paths_struct.upOut_to_aboveOut_10(end,:);
q_aboveOut = [q_aboveOut, 0.01, 0.01];


assert(all(paths_struct.downIn_to_downOut_10(1,:)-paths_struct.downIn_to_downOut_40(1,:)<0.00000001))
assert(all(paths_struct.downIn_to_downOut_40(1,:)-paths_struct.downIn_to_downOut_70(1,:)<0.00000001))


% Sanity Check that given q transforms to requested T
given_q = q_downIn;
requested_T = T_downIn;
T_calc = getTransform(panda_sc, given_q, 'panda_hand_tcp');
assert(sum(sum((T_calc-requested_T).^2))<0.001)

given_q = q_upIn;
requested_T = T_upIn;
T_calc = getTransform(panda_sc, given_q, 'panda_hand_tcp');
assert(sum(sum((T_calc-requested_T).^2))<0.001)

given_q = q_upOut;
requested_T = T_upOut;
T_calc = getTransform(panda_sc, given_q, 'panda_hand_tcp');
assert(sum(sum((T_calc-requested_T).^2))<0.001)

given_q = q_downOut;
requested_T = T_downOut;
T_calc = getTransform(panda_sc, given_q, 'panda_hand_tcp');
assert(sum(sum((T_calc-requested_T).^2))<0.001)

given_q = q_aboveOut;
requested_T = T_aboveOut;
T_calc = getTransform(panda_sc, given_q, 'panda_hand_tcp');
assert(sum(sum((T_calc-requested_T).^2))<0.001)


%     if all_valid==true
%         found_valid_cartesian_path = true;
%         combined = [all_paths{1}; all_paths{2}];
%         break;
%     else
%         combined = [];
%         all_paths = cell(1);
%     end
% assert(found_valid_cartesian_path)

% paths_struct.slot_to_above = all_paths{1};
% paths_struct.wpts_slot_to_above = all_wpts(1:9, :);
% paths_struct.above_to_out = all_paths{2};
% paths_struct.wpts_above_to_out = all_wpts(10:18, :);
% paths_struct.wpts_slot_to_above_to_out = all_wpts;

%% Joint to Joint
% 
% 
% % Paths
id = staging_data.staging_id;
paths_struct.wpts_home_to_aboveOut = joint_plan_path(panda_ec, panda_sc, env, q_home, q_aboveOut, params);
paths_struct.wpts_aboveOut_to_inter = joint_plan_path(panda_ec, panda_sc, env, q_aboveOut, staging_data.q_inter, params);
paths_struct.("wpts_aboveOut_to_inter_to_staging"+id) = [paths_struct.wpts_aboveOut_to_inter; staging_data.q_staging];



% Out to staging
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.("wpts_aboveOut_to_inter_to_staging"+id) , panda_sc,params);
paths_struct.("aboveOut_to_staging"+id+"_10") = traj_10;
paths_struct.("aboveOut_to_staging"+id+"_40")  = traj_40;
paths_struct.("aboveOut_to_staging"+id+"_70")  = traj_70;
paths_struct.("staging"+id+"_to_aboveOut_10") = traj_10_reverse;
paths_struct.("staging"+id+"_to_aboveOut_40") = traj_40_reverse;
paths_struct.("staging"+id+"_to_aboveOut_70") = traj_70_reverse;

% Append the aboveOut_to_upOut portion
paths_struct.("upOut_to_staging"+id+"_10") = [paths_struct.("upOut_to_aboveOut_10") ; paths_struct.("aboveOut_to_staging"+id+"_10")];
paths_struct.("upOut_to_staging"+id+"_40") = [paths_struct.("upOut_to_aboveOut_40") ; paths_struct.("aboveOut_to_staging"+id+"_40")];
paths_struct.("upOut_to_staging"+id+"_70") = [paths_struct.("upOut_to_aboveOut_70") ; paths_struct.("aboveOut_to_staging"+id+"_70")];
paths_struct.("staging"+id+"_to_upOut_10") = flip(paths_struct.("upOut_to_staging"+id+"_10"),1);
paths_struct.("staging"+id+"_to_upOut_40") = flip(paths_struct.("upOut_to_staging"+id+"_40"),1);
paths_struct.("staging"+id+"_to_upOut_70") = flip(paths_struct.("upOut_to_staging"+id+"_70"),1);



% Home to aboveOut
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.wpts_home_to_aboveOut, panda_sc,params);
paths_struct.home_to_aboveOut_10 = traj_10;
paths_struct.home_to_aboveOut_40 = traj_40;
paths_struct.home_to_aboveOut_70 = traj_70;
paths_struct.aboveOut_to_home_10 = traj_10_reverse;
paths_struct.aboveOut_to_home_40 = traj_40_reverse;
paths_struct.aboveOut_to_home_70 = traj_70_reverse;

% Append the aboveOut_to_upOut portion
paths_struct.home_to_upOut_10 = [paths_struct.home_to_aboveOut_10; paths_struct.aboveOut_to_upOut_10];
paths_struct.home_to_upOut_40 = [paths_struct.home_to_aboveOut_40; paths_struct.aboveOut_to_upOut_40];
paths_struct.home_to_upOut_70 = [paths_struct.home_to_aboveOut_70; paths_struct.aboveOut_to_upOut_70];
paths_struct.upOut_to_home_10 = flip(paths_struct.home_to_upOut_10,1);
paths_struct.upOut_to_home_40 = flip(paths_struct.home_to_upOut_40,1);
paths_struct.upOut_to_home_70 = flip(paths_struct.home_to_upOut_70,1);

% upOut to downOut - attempt direct motion in joint space
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, q_downOut));
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, (q_downOut+q_upOut)/2));
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, q_upOut));
paths_struct.wpts_upOut_to_downOut = [q_upOut; q_downOut]; % Direct motion;
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.("wpts_upOut_to_downOut") , panda_sc,params);
paths_struct.("upOut_to_downOut_10") = traj_10;
paths_struct.("upOut_to_downOut_40") = traj_40;
paths_struct.("upOut_to_downOut_70") = traj_70;
paths_struct.("downOut_to_upOut_10") = traj_10_reverse;
paths_struct.("downOut_to_upOut_40") = traj_40_reverse;
paths_struct.("downOut_to_upOut_70") = traj_70_reverse;


combined = [paths_struct.home_to_upOut_10; 
    paths_struct.upOut_to_downOut_10;
    paths_struct.downOut_to_downIn_10;
    paths_struct.downIn_to_upIn_10;
    paths_struct.upIn_to_upOut_10;
    paths_struct.("upOut_to_staging"+id+"_10");
    paths_struct.("staging"+id+"_to_upOut_10");
    paths_struct.upOut_to_upIn_10;
    paths_struct.upIn_to_downIn_10;
    paths_struct.downIn_to_downOut_10;
    paths_struct.downOut_to_upOut_10;
    paths_struct.upOut_to_home_10];
% plotJointMotion(panda_sc, combined, env,params)

paths_array = {paths_struct.home_to_upOut_10, 
    paths_struct.upOut_to_downOut_10,
    paths_struct.downOut_to_downIn_10,
    paths_struct.downIn_to_upIn_10,
    paths_struct.upIn_to_upOut_10,
    paths_struct.("upOut_to_staging"+id+"_10"),
    paths_struct.("staging"+id+"_to_upOut_10"),
    paths_struct.upOut_to_upIn_10,
    paths_struct.upIn_to_downIn_10,
    paths_struct.downIn_to_downOut_10,
    paths_struct.downOut_to_upOut_10,
    paths_struct.upOut_to_home_10};

for i =  1:numel(paths_array)
    arr = paths_array{i};
    arr = [arr, zeros(size(arr,1),2)];
    assert(~checkTrajForSelfCollisions(panda_sc, arr, params));
    assert(checkTrajKinematics(arr, arr(1,:),arr(end,:), params))
end
% plot_derivatives(combined, params)




% % % Trajectories
% paths_struct.home_to_out = joint_path_to_traj(paths_struct.wpts_home_to_out, params);
% paths_struct.out_to_stagingA0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingA0, params);
% paths_struct.out_to_stagingB0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingB0, params);
% paths_struct.out_to_stagingC0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingC0, params);
% paths_struct.out_to_stagingD0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingD0, params);
% paths_struct.out_to_stagingE0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingE0, params);
% 
% % % Trajectories - flipped
% % paths_struct.out_to_home = flip(paths_struct.home_to_out,1);
% paths_struct.out_to_above = flip(paths_struct.above_to_out, 1);
% paths_struct.above_to_slot = flip(paths_struct.slot_to_above,1);
% paths_struct.stagingA0_to_out = flip(paths_struct.out_to_stagingA0,1);
% paths_struct.stagingB0_to_out = flip(paths_struct.out_to_stagingB0,1);
% paths_struct.stagingC0_to_out = flip(paths_struct.out_to_stagingC0,1);
% paths_struct.stagingD0_to_out = flip(paths_struct.out_to_stagingD0,1);
% paths_struct.stagingE0_to_out = flip(paths_struct.out_to_stagingE0,1);

%  

%% Visualize
% plotJointMotion(panda_sc, comb, env, params.sphere_radius, params.sphere_origin)

%% Checks
% plot_derivatives(combined(:,1:7))
% plotJointScaled(combined(:,1:7), jointMax, jointMin)
% 
% combined_cell = {home_to_upout, out_to_above, above_to_slot, slot_to_above, above_to_out, out_to_staging , stagingto_out, out_to_above, above_to_slot, slot_to_above, above_to_out, out_to_home};
% for i=1:numel(combined_cell)
%     array = combined_cell{i};
%     assert(checkTrajectory(array, array(1,:), array(end,:), vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute))
% end









end

