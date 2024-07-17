function [all_trajectory, planned_path] = planJointToJoint(robot_ec, robot_sc, env, start, goal, params)
% Move from one joint position to another

%% Plan path
planned_path = joint_plan_path(robot_ec, robot_sc, env, start, goal, params);

%% Convert to trajectory
all_trajectory = joint_path_to_traj(planned_path, params);


%% Check
assert(checkTrajKinematics(all_trajectory, start, goal, params));

end

