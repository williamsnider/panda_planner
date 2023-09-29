% Code to move the robot into the ballpark of these estimated positions
% (will need to be verified manually)

clear; close all;
addpath("..")
params = CustomParameters();

%% Inputs
SAVE_DIR = "ballpark/";

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
[env_norm, env_big] = build_collision_environment();



%% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
weights = [1 1 1 1 1 1];
ss = manipulatorStateSpace(panda_ec); % for joint limits
stateBounds = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits


%% Plot robot
plotJointMotion(panda_sc, params.stagingB0, env, params)
plot3(params.shelf_pts(:,1),params.shelf_pts(:,2),params.shelf_pts(:,3), "r*")
