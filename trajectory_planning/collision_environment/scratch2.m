% Test if staging position is in collision with divider (move divider as
% close as possible).

%% Clear all variables and load parameters
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
date_prefix = "20250620_";
slot_dir = "20250618_manual_slots";


%% Load stagingA0a position
fname = "/home/oconnorlab/Code/panda_planner/trajectory_planning/quartets/trajectories/20241009_A.mat";
m = load(fname);
m = m.data_struct;

% Load staging A0a
q_stagingA0a = m.staging_arr(1,:);

% Plot
plotJointMotion(panda_sc, q_stagingA0a, env, params); hold on;
show(panda_sc, q_stagingA0a, "Collisions", "on")

assert(~any(checkCollision(panda_ec, q_stagingA0a, env)))
