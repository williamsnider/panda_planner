clear; close all;
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/add_all_paths.m
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/parameters.m
format long

% Inputs
start = [-0.0121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.01, 0.01];
goal = [-0.1121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.01,0.01];

% Load robot
panda = loadPandaWithShape();

% Load collision environment
env = build_collision_environment();
env = env(440:end);
env = {};

% Plan
scaleList = [0.01, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80];
for i = 1:numel(scaleList)
    
    vScale = scaleList(i);
    vMaxAll = vMaxAllAbsolute*vScale;
    all_trajectory = planJointToJoint(panda, env, start,goal, vMaxAll, aMaxAll, jMaxAll);
    
    
    % Export as csv
    savename = ['joint1_slow_',num2str(vScale*100),'%.csv'];
    writematrix(all_trajectory(:, 1:7), savename)

end

for i = 1:numel(scaleList)

    % Load csv
    vScale = scaleList(i);
    savename = ['joint1_slow_',num2str(vScale*100),'%.csv']
    q = readmatrix(savename);

    % Append fake data for gripper
    num_steps = size(q,1);
    q = [q, ones(num_steps,1)*0.01, ones(num_steps,1)*0.01];

    assert(checkTrajectory(q, start, goal, vMaxAll, aMaxAll, jMaxAll))
    
    % Visualize
    close all;
    figure;
    plotJointMotion(panda, q, env)

end
