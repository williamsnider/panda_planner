function [panda_ec_orig, panda_sc_orig, panda_ec_A, panda_sc_A, ik_orig, ik_A, env, body_names, theta_list, X_SHIFT_EXTREME_TO_STAGING] = load_quartet_common_variables(params)



%% Inputs
theta_list = 0:pi/2:3*pi/2;
JOINT_REDUCTION = 0.3;  % Safety cutoffs for J1-6 for extreme position (ensures not near joint limits)
J7_REDUCTION = 0.2;  % Safety cutoff for J7 (have to be more generous because we need 270deg of rotation)
% W_SHIFT = [0.00, 0, 0.00];  % Translation from A to W in extreme position (plus a rotation, as in calc_TA_TW);
X_SHIFT_EXTREME_TO_STAGING = 0.15;


%% Load robot with quartert
[panda_ec_orig, panda_sc_orig] = loadPandaWithShape(params);
[panda_ec_A, panda_sc_A] = loadPandaWithShape(params);
% [panda_ec_W, panda_sc_W] = loadPandaWithShape(params);
env = build_collision_environment;


% Joints 1-6
for body_num = 1:6
    oldLimits = panda_sc_orig.Bodies{body_num}.Joint.PositionLimits;
    newLimits = oldLimits + [JOINT_REDUCTION,-JOINT_REDUCTION];
    panda_sc_A.Bodies{body_num}.Joint.PositionLimits = newLimits;
%     panda_sc_W.Bodies{body_num}.Joint.PositionLimits = newLimits;

end

% Joint 7
body_num = 7;
oldLimits = panda_sc_orig.Bodies{body_num}.Joint.PositionLimits;
newLimits = oldLimits + [J7_REDUCTION,-J7_REDUCTION];
panda_sc_A.Bodies{body_num}.Joint.PositionLimits = newLimits;
% panda_sc_W.Bodies{body_num}.Joint.PositionLimits = newLimits;

% Restrict joint 1 to be positive
body_num = 1;
oldLimits = panda_sc_A.Bodies{body_num}.Joint.PositionLimits;
newLimits = [0, oldLimits(2)];
panda_sc_A.Bodies{body_num}.Joint.PositionLimits = newLimits;
% panda_sc_W.Bodies{body_num}.Joint.PositionLimits = newLimits;


% % Restrict joint 2 to be positive for panda_A
% body_num = 2;
% oldLimits = panda_sc_A.Bodies{body_num}.Joint.PositionLimits;
% newLimits = [0, oldLimits(2)];
% panda_sc_A.Bodies{body_num}.Joint.PositionLimits = newLimits;


% Restrict joint 2 to be negative for panda_A
body_num = 2;
oldLimits = panda_sc_A.Bodies{body_num}.Joint.PositionLimits;
newLimits = [oldLimits(1), 0];
panda_sc_A.Bodies{body_num}.Joint.PositionLimits = newLimits;

%% Set up inverse kinematics
ik_A = inverseKinematics('RigidBodyTree',panda_sc_A);
ik_A.SolverParameters.MaxIterations = 1000;
% ik_W = inverseKinematics('RigidBodyTree',panda_sc_W);
% ik_W.SolverParameters.MaxIterations = 1000;
ik_orig = inverseKinematics('RigidBodyTree', panda_sc_orig);
ik_orig.SolverParameters.MaxIterations = 1000;


% Get list of quartet shape names
body_names = {};
for i = panda_sc_orig.NumBodies-3:panda_sc_orig.NumBodies
    body_names{end+1} = panda_sc_orig.BodyNames{i};
end
