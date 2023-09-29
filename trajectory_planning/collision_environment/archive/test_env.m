clear; close all;
run ../../parameters.m

%% Inputs
ABOVE_HEIGHT = 0.0202; %m; how high the shape is pulled when picking
OUT_DIST = 0.130; %m; how far out the shape is pulled when picking
SHAPE_ROTATION_ABOUT_Z = pi/18;

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();
% env(end) = []; disp('removing monkey grasp region')

show(panda_ec, q_home)
plotJointMotion(panda_ec, q_home, env, sphere_radius, sphere_origin)% 
hold on;


% Shelf inputs
th = linspace(3*pi/4, -3*pi/4, 50);

% Points
pts = [];
for Z_num = 1:numel(Z_list)

    % Get Z-height and radius of shelf
    Z = Z_list(Z_num); 
    radius = radius_list(Z_num) - (0.0254/2 + 0.015); % Correct for interior thickness of shelf

    % Calculate x,y,z
    x = radius*cos(th);
    y = radius*sin(th);
    z = repelem([Z], numel(th));
    pts = [pts;[x',y',z']];
end
plot3(pts(:,1),pts(:,2),pts(:,3),"k*")


% %% Set up inverse kinematics
% ik = inverseKinematics('RigidBodyTree',panda_sc);
% ik.SolverParameters.MaxIterations = 1000;
% weights = [0.25 0.25 0.25 1 1 1];
% ss = manipulatorStateSpace(panda_ec); % for joint limits