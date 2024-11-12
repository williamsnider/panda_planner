%% Clear all variables and loam parameters
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Load variables
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment;
ik = inverseKinematics('RigidBodyTree', panda_sc);
ik.SolverParameters.MaxIterations = 1000;

quartet_fname = "/home/oconnorlabmatlab/Code/panda_planner/trajectory_planning/quartets/trajectories/20241009_A.mat";
save_dir = "/home/oconnorlabmatlab/Code/panda_planner/trajectory_planning/slots/trajectories/";
date_prefix = "20241009_";
slot_dir = "20241112_manual_slots";

% quartet_slots = {
%     "12C32",
%     "12C20",
%     "12C08"}
calcSlot(panda_ec, panda_sc, env, ik, slot_name, slot_dir, date_prefix, save_dir, quartet_fname, params)