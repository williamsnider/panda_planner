%% Clear all variables and loam parameters
clear; close all;
addpath("..")
params = CustomParameters();
warning('off', 'all');

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment;

%% Load csv
csvpath = '/home/oconnorlabmatlab/Code/libfranka/oconnor/trajectories_old/useful/20240820_home_to_stagingD1a_10%.csv';
plotCSV(panda_sc, csvpath, env, params);
% 
% csvpath = '/home/oconnorlabmatlab/Code/libfranka/oconnor/trajectories/A/trajectories/20241008_stagingA1a_to_stagingD1b_10%.csv';
% plotCSV(panda_sc, csvpath, env, params);


traj  = readmatrix(csvpath);
plot_derivatives(traj, params);

waitforbuttonpress;


% Compare location of whack_a_mole and tcp
T1 = getTransform(panda_sc, [traj(end,:), 0.01 ,0.01], 'panda_hand_tcp')

