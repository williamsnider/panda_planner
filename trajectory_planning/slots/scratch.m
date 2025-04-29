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
save_dir = params.CustomParametersDir+"/trajectory_planning/slots/trajectories/";
quartet_slots_csv = params.CustomParametersDir+"/trajectory_planning/slots/quartet_slots.csv";
date_prefix = "20250418_";
slot_dir = "20241112_manual_slots";

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% Create pivot point
q_slot = [-0.648525,0.836017,0.202194,-1.47265,-2.87303,2.44462,0.424414, 0.01, 0.01];
T_slot = getTransform(panda_sc, q_slot, "panda_hand_tcp");
q_downIn = q_slot;
dist_hori = 0.004;
dist_vert = 0.000;

% Calculate T_pivot
T_pivot = T_slot;
T_pivot(1:3,4) = T_pivot(1:3,4) + T_pivot(1:3,3)*dist_hori;
T_pivot(1:3,4) = T_pivot(1:3,4) + T_pivot(1:3,2)*dist_vert;

initialGuess = q_slot;
weights = [1 1 1 1 1 1];
[q_pivot,solnInfo] = ik('panda_hand_tcp',T_pivot,weights,initialGuess);
assert(strcmp(solnInfo.Status,'success'))

% Create direct motion path
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, q_downIn));
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, (q_downIn+q_pivot)/2));
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, q_pivot));
paths_struct = struct();
paths_struct.wpts_downIn_to_pivot = [q_downIn; q_pivot]; % Direct motion;
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.("wpts_downIn_to_pivot") , panda_sc,params);
paths_struct.("downIn_to_pivot_10") = traj_10;
paths_struct.("downIn_to_pivot_40") = traj_40;
paths_struct.("downIn_to_pivot_70") = traj_70;
paths_struct.("pivot_to_downIn_10") = traj_10_reverse;
paths_struct.("pivot_to_downIn_40") = traj_40_reverse;
paths_struct.("pivot_to_downIn_70") = traj_70_reverse;

combined = [paths_struct.downIn_to_pivot_10; paths_struct.pivot_to_downIn_10];
plotJointMotion(panda_sc, combined, env,params)
