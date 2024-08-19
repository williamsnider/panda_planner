%% Clear all variables and loam parameters
clear; close all;
addpath("..")
params = CustomParameters();
warning('off', 'all');

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment;

% %% Load csv
% csvpath = '/home/oconnorlabmatlab/Code/libfranka/oconnor/trajectories/useful/20240728_home_to_stagingY1a_10%.csv';
% plotCSV(panda_sc, csvpath, env, params);

csvpath = '/home/oconnorlabmatlab/Code/libfranka/oconnor/trajectories/W/trajectories/20240728_stagingW1a_to_stagingW1b_10%.csv';
plotCSV(panda_sc, csvpath, env, params);


traj  = readmatrix(csvpath);
plot_derivatives(traj, params);
