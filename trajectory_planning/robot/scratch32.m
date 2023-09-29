close all;
clear;
rob = importrobot('panda_description/panda_edited.urdf', 'DataFormat','row');

config = [-0.0121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306+pi/4, 0.01, 0.01];
% clearCollision(rob.Bodies{2})
show(rob, config, 'Collisions', 'on', 'Visuals', 'off');


[isColliding, sepDist] = checkCollision(rob, config, "Exhaustive", "on", 'SkippedSelfCollisions','Parent');

[b1, b2] = find(isnan(sepDist));
rob.BodyNames{b1}
rob.BodyNames{b2}

T = getTransform(rob, config, 'panda_hand');
xyz = T(1:3, 4)';
hold on;
plot3(xyz(1), xyz(2), xyz(3), 'ro', 'MarkerSize', 50)