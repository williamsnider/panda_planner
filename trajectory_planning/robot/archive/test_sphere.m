clear; close all;
rob = loadPandaWithShape();

start = [-0.0121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.04, 0.04];
goal = [-0.1121707,-0.561084,0.00127942,-1.60702,-0.0211893,2.03285,0.802306, 0.04,0.04];


% Custom class for checking if within sphere
env = {};
rrt = manipulatorRRTSphere(rob,env);
rrt.SkippedSelfCollisions = "parent";  % prevents warning
tic
path = plan(rrt, start, goal);
toc

% Check self collision in a configuration
[isColliding, sepDist] = checkCollision(rob, start, "Exhaustive", "on", 'SkippedSelfCollisions','Parent');

[b1, b2] = find(isnan(sepDist));
rob.BodyNames{b1}
rob.BodyNames{b2}

T = getTransform(rob, start, 'panda_hand');
xyz = T(1:3, 4)';
show(rob, start, 'Collisions', 'off');
hold on;
plot3(xyz(1), xyz(2), xyz(3), 'ro', 'MarkerSize', 50);

addpath([fileparts(mfilename('fullpath')), '/../../visualization'])
plotJointMotion(panda,smoothed_q',{})
