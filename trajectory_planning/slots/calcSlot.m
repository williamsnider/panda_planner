%% Clear all variables and loam parameters
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');
quartet_fname = "/home/oconnorlabmatlab/Code/panda_planner/trajectory_planning/quartets/trajectories/20241009_A.mat";

% Load variables
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment;
ik = inverseKinematics('RigidBodyTree', panda_sc);
ik.SolverParameters.MaxIterations = 1000;

q_slot = [-0.648525,0.836017,0.202194,-1.47265,-2.87303,2.44462,0.424414];
q_slot = [q_slot, 0.01, 0.01];

slot_name = "04B44";
SAVE_DIR = "./";
savename = SAVE_DIR+"missing"+slot_name;
stateBounds = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits

% Create struct containing stagingA0a data
A = load(quartet_fname);
A = A.data_struct;
staging_data = struct();
idx = 1;
staging_data.q_staging = A.staging_arr(idx,:);
staging_data.q_inter = A.inter_arr(idx,:);
staging_data.cell_staging_to_inter_path = A.cell_staging_to_inter_path{idx};
paths_struct = calcSlotTrajectories(panda_ec, panda_sc, env, ik, q_slot, savename, stateBounds, staging_data, params)