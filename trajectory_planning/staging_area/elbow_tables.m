% Generate a look-up table for the elbow angle between each staging and
% extreme position

%% Inputs
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Set random stream for reproducibility
stream = RandStream('mt19937ar', 'Seed', 123); 
RandStream.setGlobalStream(stream);

TRAVEL_DIST = 0.15;
JOINT_REDUCTION = 0.3;
SAVE_DIR = "./elbow_tables/20240424_";


%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();

for body_num = 1:6
    oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
    newLimits = oldLimits + [JOINT_REDUCTION,-JOINT_REDUCTION];
    panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;
end

ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
weights = [1 1 1 1 1 1];


%% Calculate elbow table
letter = "A";
q_extreme = getfield(params, "extreme_"+letter+"0");
[elbow_table, q_staging] = calc_elbow_table(q_extreme, panda_sc, TRAVEL_DIST, ik, SAVE_DIR, letter);





show(panda_sc, q_arr(1,:)); hold on;
show(panda_sc, q_arr(end,:));