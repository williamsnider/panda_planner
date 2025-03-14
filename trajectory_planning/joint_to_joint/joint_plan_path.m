function planned_path = joint_plan_path(robot_ec, robot_sc, env, start, goal, params)
%JOINT_PLAN_PATHS Summary of this function goes here
%   Detailed explanation goes here
%% Create collision validator
sv = construct_state_validator(robot_ec, robot_sc, env, params);

% Test start and goal position
assert(sv.isStateValid(start), "Starting joint position is invalid, probably outside the spherical region.")
% plotJointMotion(robot_sc, start, env, params)
assert(sv.isStateValid(goal), "Goal joint position is invalid, probably outside the spherical region.")
% plotJointMotion(robot_sc, goal, env, params)


% Create simple linear path
[planned_path, IsDirectValid] = check_direct_path(sv, start, goal);

%% Plan path using RRT
if IsDirectValid == false
    rrt = manipulatorRRTSphere(robot_ec, robot_sc, env, params);
    rrt.SkippedSelfCollisions = "parent";
    planned_path = plan(rrt,start,goal);

end

%% Shorten path

% Reduce the number of waypoints
if size(planned_path,1) > 2
    parts = [1]; % Index of waypoints in planned_path that are valid motions
    while parts(end) < size(planned_path,1)

        % Work backwards for final position until the start -> final motion is
        % valid
        for f = size(planned_path,1):-1:parts(end)+1
            sub_start = planned_path(parts(end),:);
            sub_goal = planned_path(f,:);
            if sv.isMotionValid(sub_start, sub_goal)
                parts = [parts,f];
                break
            end
        end
    end
    planned_path = planned_path(parts,:);  % Select only necessary waypoints
end
end

