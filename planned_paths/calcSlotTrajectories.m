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

ss = ManipulatorStateSpaceSphere(panda_ec, panda_sc);
sv = ManipulatorStateValidatorSphere(ss, env, params.validationDistance,params.radius_offset, params);
sv.IgnoreSelfCollision = false;
sv.Environment = env;

%% Calculate poses for pick/place
T_slot = getTransform(panda_sc, q_slot, "panda_hand_tcp");

T_above = T_slot;
T_above(3,4) = T_slot(3,4) + ABOVE_HEIGHT;

% Calculate minimum OUT_DIST to pull shape within sphere
for OUT_DIST = 0.0:0.005:0.4

    % Calculate T_out
    T_out = T_slot;
    T_out(1:3,4) = T_out(1:3,4) - T_out(1:3,3)*OUT_DIST;

    % Do inverse kinematics
    initialGuess = q_slot;
    weights = [1 1 1 1 1 1];
    [q,solnInfo] = ik('panda_hand_tcp',T_out,weights,initialGuess);

    % Break loop if valid (inside sphere)
    if sv.isStateValid(q)
        break
    end
end
assert(sv.isStateValid(q))
% 
% show(panda_sc, q); hold on;
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
T_array = cat(3,T_slot, T_above, T_out);
found_valid_cartesian_path = false;

% Use known q for q_slot
q = q_slot;

%Redo inverse kinematics if q is outside joint limits or in collision
q = round(q, 10);  % Rounding ensures not exceeding joint limits by precision error

%     if ~custom_check_valid_state(panda_ec, panda_sc, env, q, stateBounds)
%         continue
%     end

% 
vScale = 0.1;
[all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_array,q,params);
assert(all_valid)
paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);

vScale = 0.4;
[all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_array,q,params);
assert(all_valid)
paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);


vScale = 0.7;
[all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_array,q,params);
assert(all_valid)
paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);


% Ensure out joint position identical
assert(all(paths_struct.above_to_out_10(end,:)-paths_struct.above_to_out_40(end,:)<0.00000001))
assert(all(paths_struct.above_to_out_40(end,:)-paths_struct.above_to_out_70(end,:)<0.00000001))
q_out = paths_struct.above_to_out_10(end,:);
q_out = [q_out, 0.01, 0.01];

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
paths_struct.wpts_home_to_out = joint_plan_path(panda_ec, panda_sc, env, q_home, q_out, params);
paths_struct.wpts_out_to_inter = joint_plan_path(panda_ec, panda_sc, env, q_out, staging_data.q_inter, params);
paths_struct.("wpts_out_to_inter_to_staging"+id) = [paths_struct.wpts_out_to_inter; staging_data.q_staging];
% paths_struct.wpts_out_to_stagingA0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingA0, params);
% paths_struct.wpts_out_to_stagingB0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingB0, params);
% paths_struct.wpts_out_to_stagingC0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingC0, params);
% paths_struct.wpts_out_to_stagingD0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingD0, params);
% paths_struct.wpts_out_to_stagingE0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingE0, params);

% Trajectories

% Out to staging
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.("wpts_out_to_inter_to_staging"+id) , panda_sc,params);
paths_struct.("out_to_staging"+id+"_10") = traj_10;
paths_struct.("out_to_staging"+id+"_40")  = traj_40;
paths_struct.("out_to_staging"+id+"_70")  = traj_70;
paths_struct.("staging"+id+"_to_out_10") = traj_10_reverse;
paths_struct.("staging"+id+"_to_out_40") = traj_40_reverse;
paths_struct.("staging"+id+"_to_out_70") = traj_70_reverse;

% Home to out
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.wpts_home_to_out, panda_sc,params);
paths_struct.home_to_out_10 = traj_10;
paths_struct.home_to_out_40 = traj_40;
paths_struct.home_to_out_70 = traj_70;
paths_struct.out_to_home_10 = traj_10_reverse;
paths_struct.out_to_home_40 = traj_40_reverse;
paths_struct.out_to_home_70 = traj_70_reverse;


% combined = [paths_struct.home_to_out_10; paths_struct.out_to_above_10; paths_struct.above_to_slot_10;paths_struct.slot_to_above_10;paths_struct.above_to_out_10;paths_struct.out_to_staging_10;paths_struct.staging_to_out_10;paths_struct.out_to_above_10;paths_struct.above_to_slot_10;paths_struct.slot_to_above_10;paths_struct.above_to_out_10;paths_struct.out_to_home_10];
% plotJointMotion(panda_sc, combined, env,params)

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
% combined_cell = {home_to_out, out_to_above, above_to_slot, slot_to_above, above_to_out, out_to_staging , stagingto_out, out_to_above, above_to_slot, slot_to_above, above_to_out, out_to_home};
% for i=1:numel(combined_cell)
%     array = combined_cell{i};
%     assert(checkTrajectory(array, array(1,:), array(end,:), vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute))
% end









end

