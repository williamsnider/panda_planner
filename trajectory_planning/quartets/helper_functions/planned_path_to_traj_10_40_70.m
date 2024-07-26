function [traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(planned_path, panda_sc,params)
%PLANNED_PATH_TO_TRAJ_10_40_70 Summary of this function goes here

% Calculate 70%
params_copy = params;
params_copy.vScale = 0.7;
params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
traj_70 = joint_path_to_traj(planned_path, params_copy);
assert(checkTrajKinematics(traj_70, planned_path(1,:), planned_path(end,:), params_copy)); % Check
assert(~checkTrajForSelfCollisions(panda_sc, traj_70, params));
traj_70_reverse = flip(traj_70,1);
assert(checkTrajKinematics(traj_70_reverse, planned_path(end,:), planned_path(1,:), params_copy)); % Check

% Calculate 10%
params_copy = params;
params_copy.vScale = 0.1;
params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
traj_10 = joint_path_to_traj(planned_path, params_copy);
assert(checkTrajKinematics(traj_10, planned_path(1,:), planned_path(end,:), params_copy)); % Check
assert(~checkTrajForSelfCollisions(panda_sc, traj_10, params));
traj_10_reverse = flip(traj_10,1);
assert(checkTrajKinematics(traj_10_reverse, planned_path(end,:), planned_path(1,:), params_copy)); % Check

% Calculate 40%
params_copy = params;
params_copy.vScale = 0.4;
params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
traj_40 = joint_path_to_traj(planned_path, params_copy);
assert(checkTrajKinematics(traj_40, planned_path(1,:), planned_path(end,:), params_copy)); % Check
assert(~checkTrajForSelfCollisions(panda_sc, traj_40, params));
traj_40_reverse = flip(traj_40,1);
assert(checkTrajKinematics(traj_40_reverse, planned_path(end,:), planned_path(1,:), params_copy)); % Check


% Remove joints 8 and 9
traj_10 = traj_10(:,1:7);
traj_40 = traj_40(:,1:7);
traj_70 = traj_70(:,1:7);
traj_10_reverse = traj_10_reverse(:,1:7);
traj_40_reverse = traj_40_reverse(:,1:7);
traj_70_reverse = traj_70_reverse(:,1:7);
end

