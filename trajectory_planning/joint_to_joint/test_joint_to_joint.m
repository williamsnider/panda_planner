format long

addpath([pwd, '/../..'])
run parameters.m

% Inputs
start = [-0.0121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.01, 0.01];
goal = [-0.1121707,-0.561084,1.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.01,0.01];

% Load robot
addpath([fileparts(mfilename('fullpath')), '/../robot'])
panda = loadPandaWithShape();

% Load collision environment
addpath([fileparts(mfilename('fullpath')), '/../collision_environment'])
env = build_collision_environment();
env = env(440:end);
% env = {}

% Plan
tic
all_trajectory = planJointToJoint(panda, env, start,goal, vMaxAll, aMaxAll, jMaxAll);
toc

% Visualize
addpath([fileparts(mfilename('fullpath')), '/../../visualization'])
plotJointMotion(panda, all_trajectory', env)

% Export as csv
writematrix(all_trajectory(1:7, :)', 'pickandplace_angles.csv')

