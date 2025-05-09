clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Load variables
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment;
ik = inverseKinematics('RigidBodyTree', panda_sc);
ik.SolverParameters.MaxIterations = 1000;

quartet_fname = params.CustomParametersDir+"/trajectory_planning/quartets/trajectories/20241009_A.mat";
save_dir = params.CustomParametersDir+"/trajectory_planning/slots/close/";
quartet_slots_csv = params.CustomParametersDir+"/trajectory_planning/slots/quartet_slots.csv";
date_prefix = "20250506_";
slot_dir = "20250508_manual_slots";

A = load(quartet_fname);
A = A.data_struct;
staging_data = struct();
idx = 1;  % Corresponds to staging id A0a;
staging_data.q_staging = A.staging_arr(idx,:);
staging_data.staging_id = A.names{idx};
staging_data.q_inter = A.inter_arr(idx,:);
staging_data.cell_staging_to_inter_path = A.cell_staging_to_inter_path{idx};
q_home = params.q_home;

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

slot_id_list = {'08C42', '06A52', '06C20', '09A30', '04C32', '05C32', '07C46','08A18','09A18'};

i = 9;
slot_name = slot_id_list{i};

% Read T
slot_fname = strcat(pwd,"/",slot_dir, "/",slot_name+".txt");
if strcmp(slot_name, '06A52')
    [~, val1] = readSlot(strcat(pwd,"/",slot_dir, "/","06A50"+".txt"));
    [~, val2] = readSlot(strcat(pwd,"/",slot_dir, "/","06B00"+".txt"));
    T_downIn1 = reshape(val1.O_T_EE,4,4);
    T_downIn2 = reshape(val2.O_T_EE,4,4);
    T_downIn = transformtraj(T_downIn1, T_downIn2, [0, 1], 0.5);
elseif any([strcmp(slot_name, '07C46'), strcmp(slot_name, '08A18'), strcmp(slot_name, '09A18')])
    [q_downIn_orig, val] = readSlot(slot_fname);
    T_downIn_orig = reshape(val.O_T_EE,4,4);
    T = T_downIn_orig;
    axis_dir = T(1:3, 2);
    point_on_axis = T(1:3, 4);
    
    theta = deg2rad(-3);
    v = axis_dir / norm(axis_dir);
    
    K = [   0     -v(3)   v(2);
           v(3)    0    -v(1);
          -v(2)   v(1)    0 ];
    
    R = eye(3) + sin(theta)*K + (1 - cos(theta))*(K*K);
    
    T_rot = eye(4);
    T_rot(1:3,1:3) = R;
    T_rot(1:3,4) = point_on_axis - R * point_on_axis;
    
    T_downIn = T_rot * T;
else
    [q_downIn, val] = readSlot(slot_fname);
    T_downIn = reshape(val.O_T_EE,4,4);
end

% Test random q, see if it works with sequences needed
% initialGuess = q_downIn;
for j=1:1
initialGuess = randomConfiguration(panda_ec);
weights = [1 1 1 1 1 1];
[q_downIn_ik,solnInfo] = ik('panda_hand_tcp',T_downIn,weights,initialGuess);
T_downIn_ik = getTransform(panda_sc, q_downIn_ik, "panda_hand_tcp");

% q_downIn
q_downIn_ik
solnInfo
end

q_slot = q_downIn_ik 
ik.SolverParameters.SolutionTolerance = 0.0045;

ss = ManipulatorStateSpaceEllipsoid(panda_ec, panda_sc);
sv = ManipulatorStateValidatorEllipsoid(ss, env, params.validationDistance,params.ellipsoid_radius_offset, params);
sv.IgnoreSelfCollision = false;
sv.Environment = env;

T_upIn = T_downIn;
ABOVE_HEIGHT = params.ABOVE_HEIGHT;
T_upIn(3,4) = T_upIn(3,4) + ABOVE_HEIGHT;

% Calculate minimum OUT_DIST to pull shape within sphere
for OUT_DIST = 0.0:0.005:0.4

    % Calculate T_upOut
    T_upOut = T_upIn;
    T_upOut(1:3,4) = T_upOut(1:3,4) - T_upOut(1:3,3)*OUT_DIST;

%     % Calculate T_out
%     T_out = T_slot;
%     T_out(1:3,4) = T_out(1:3,4) - T_out(1:3,3)*OUT_DIST;

    % Do inverse kinematics
    initialGuess = q_slot;
    weights = [1 1 1 1 1 1];
    [q_upOut,solnInfo] = ik('panda_hand_tcp',T_upOut,weights,initialGuess);

    % Break loop if valid (inside ellipsoidf for both T_upOut and T_downOut)
    if sv.isStateValid(q_upOut)

        % Test if q_downOut valid too
        T_downOut = T_upOut;
        T_downOut(3,4)= T_downIn(3,4);
        [q_downOut,solnInfo] = ik('panda_hand_tcp',T_downOut,weights,q_upOut);
        if sv.isStateValid(q_downOut)
            break
        end
    end

    if  OUT_DIST == 0.2
%         plotJointMotion(panda_sc, q_upOut, env, params)
%         disp('here')
    end
end

% plotJointMotion(panda_sc, q_upOut, env, params)
% disp('here')
assert(sv.isStateValid(q_upOut))


% T_downIn = T_slot;
T_downOut = T_upOut;
T_downOut(3,4)= T_downIn(3,4);


% Check Z's 
assert(abs(T_downOut(3,4)-T_downIn(3,4))<0.015)
assert(abs(T_upOut(3,4)-T_upIn(3,4))<0.02)


T_downIn_to_downOut = cat(3, T_downIn, T_downOut);



%% Calculate trajectories
paths_struct = struct();

% Cartesian paths for pick/place
T_downIn_to_upIn_to_upOut = cat(3,T_downIn, T_upIn, T_upOut);
found_valid_cartesian_path = false;

% Use known q for q_slot
q_downIn = q_slot;

%Redo inverse kinematics if q is outside joint limits or in collision
q_downIn = round(q_downIn, 10);  % Rounding ensures not exceeding joint limits by precision error



% Calculate T_pivot and q_pivot
dist_hori = 0.004;
dist_vert = 0.0015;
T_pivot = T_downIn;
T_pivot(1:3,4) = T_pivot(1:3,4) + T_pivot(1:3,3)*dist_hori;
T_pivot(1:3,4) = T_pivot(1:3,4) - T_pivot(1:3,1)*dist_vert;% Negative since sign is flipped for this vector
initialGuess = q_downIn;
weights = [1 1 1 1 1 1];
[q_pivot,solnInfo] = ik('panda_hand_tcp',T_pivot,weights,initialGuess);
assert(strcmp(solnInfo.Status,'success'))


stateBounds=ss.JointBounds';
% 
vScale = 0.1;
[all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_upIn_to_upOut,q_downIn,params);
assert(all_valid)
[downIn_to_downOut_path, downIn_to_downOut_valid, downIn_to_downOut_all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_downOut,q_downIn,params);
assert(downIn_to_downOut_valid)
all_paths{3} = downIn_to_downOut_path{1};
all_wpts(end+1:end+9,:) = downIn_to_downOut_all_wpts;
paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);


vScale = 0.4;
[all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_upIn_to_upOut,q_downIn,params);
assert(all_valid)
[downIn_to_downOut_path, downIn_to_downOut_valid, downIn_to_downOut_all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_downOut,q_downIn,params);
assert(downIn_to_downOut_valid)
all_paths{3} = downIn_to_downOut_path{1};
all_wpts(end+1:end+9,:) = downIn_to_downOut_all_wpts;
paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);

vScale = 0.7;
[all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_upIn_to_upOut,q_downIn,params);
assert(all_valid)
[downIn_to_downOut_path, downIn_to_downOut_valid, downIn_to_downOut_all_wpts] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, vScale,stateBounds, T_downIn_to_downOut,q_downIn,params);
assert(downIn_to_downOut_valid)
all_paths{3} = downIn_to_downOut_path{1};
all_wpts(end+1:end+9,:) = downIn_to_downOut_all_wpts;
paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale);



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

given_q = q_pivot;
requested_T = T_pivot;
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
paths_struct.wpts_home_to_upOut = joint_plan_path(panda_ec, panda_sc, env, q_home, q_upOut, params);
paths_struct.wpts_upOut_to_inter = joint_plan_path(panda_ec, panda_sc, env, q_upOut, staging_data.q_inter, params);
paths_struct.("wpts_upOut_to_inter_to_staging"+id) = [paths_struct.wpts_upOut_to_inter; staging_data.q_staging];

paths_struct.wpts_upOut_to_downOut = joint_plan_path(panda_ec, panda_sc, env, q_upOut, q_downOut, params);
% paths_struct.wpts_out_to_stagingA0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingA0, params);
% paths_struct.wpts_out_to_stagingB0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingB0, params);
% paths_struct.wpts_out_to_stagingC0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingC0, params);
% paths_struct.wpts_out_to_stagingD0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingD0, params);
% paths_struct.wpts_out_to_stagingE0 = joint_plan_path(panda_ec, panda_sc, env, q_out, params.stagingE0, params);

% Trajectories

% upOut to downOut
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.("wpts_upOut_to_downOut") , panda_sc,params);
paths_struct.("upOut_to_downOut_10") = traj_10;
paths_struct.("upOut_to_downOut_40") = traj_40;
paths_struct.("upOut_to_downOut_70") = traj_70;
paths_struct.("downOut_to_upOut_10") = traj_10_reverse;
paths_struct.("downOut_to_upOut_40") = traj_40_reverse;
paths_struct.("downOut_to_upOut_70") = traj_70_reverse;


% Out to staging
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.("wpts_upOut_to_inter_to_staging"+id) , panda_sc,params);
paths_struct.("upOut_to_staging"+id+"_10") = traj_10;
paths_struct.("upOut_to_staging"+id+"_40")  = traj_40;
paths_struct.("upOut_to_staging"+id+"_70")  = traj_70;
paths_struct.("staging"+id+"_to_upOut_10") = traj_10_reverse;
paths_struct.("staging"+id+"_to_upOut_40") = traj_40_reverse;
paths_struct.("staging"+id+"_to_upOut_70") = traj_70_reverse;

% Home to upOut
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.wpts_home_to_upOut, panda_sc,params);
paths_struct.home_to_upOut_10 = traj_10;
paths_struct.home_to_upOut_40 = traj_40;
paths_struct.home_to_upOut_70 = traj_70;
paths_struct.upOut_to_home_10 = traj_10_reverse;
paths_struct.upOut_to_home_40 = traj_40_reverse;
paths_struct.upOut_to_home_70 = traj_70_reverse;


% downIn to pivot - attempt direct motion in joint space
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, q_downIn));
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, (q_downIn+q_pivot)/2));
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, q_pivot));
paths_struct.wpts_downIn_to_pivot = [q_downIn; q_pivot]; % Direct motion;
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.("wpts_downIn_to_pivot") , panda_sc,params);
paths_struct.("downIn_to_pivot_10") = traj_10;
paths_struct.("downIn_to_pivot_40") = traj_40;
paths_struct.("downIn_to_pivot_70") = traj_70;
paths_struct.("pivot_to_downIn_10") = traj_10_reverse;
paths_struct.("pivot_to_downIn_40") = traj_40_reverse;
paths_struct.("pivot_to_downIn_70") = traj_70_reverse;

combined = [paths_struct.home_to_upOut_10; 
    paths_struct.upOut_to_downOut_10;
    paths_struct.downOut_to_downIn_10;
    paths_struct.downIn_to_pivot_10;
    paths_struct.pivot_to_downIn_10;
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

paths_array = {paths_struct.home_to_upOut_10; 
    paths_struct.upOut_to_downOut_10;
    paths_struct.downOut_to_downIn_10;
    paths_struct.downIn_to_upIn_10;
    paths_struct.downIn_to_pivot_10;
    paths_struct.pivot_to_downIn_10;
    paths_struct.upIn_to_upOut_10;
    paths_struct.("upOut_to_staging"+id+"_10");
    paths_struct.("staging"+id+"_to_upOut_10");
    paths_struct.upOut_to_upIn_10;
    paths_struct.upIn_to_downIn_10;
    paths_struct.downIn_to_downOut_10;
    paths_struct.downOut_to_upOut_10;
    paths_struct.upOut_to_home_10};

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






% Write paths_struct fields to csv
traj_name_list = fieldnames(paths_struct);
for i = 1:numel(traj_name_list)
    traj_name = traj_name_list{i};
    
    % Skip wpts
    if contains(traj_name,"wpts")
        continue
    end

    % Write to csv
    traj = paths_struct.(traj_name);
    savename = strcat(save_dir, date_prefix, slot_name,"_",traj_name,"%.csv");
    
    writematrix(paths_struct.(traj_name), savename)
end
disp("Finished "+slot_name)




