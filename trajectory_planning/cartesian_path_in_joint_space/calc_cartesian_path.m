function [q, wpts, valid] = calc_cartesian_path(robot_ec, robot_sc, env, jointLimits, startPose,goalPose,  q_start, params)

%% Get waypoints along cartesian path
tvec = linspace(0,1,params.num_waypoints);
[T,~,~] = transformtraj(startPose, goalPose, [0,1], tvec);


wpts = zeros(9, params.num_waypoints);
wpts(:,1) = q_start;  % Assign first waypoint as inputted q_start (forces consecutive cartesian paths to start/end at same pose)
ik = inverseKinematics('RigidBodyTree', robot_ec,'SolverAlgorithm', 'LevenbergMarquardt');
ik.SolverParameters.MaxIterations = 2500;
weights = [1 1 1 1 1 1];
previous_q = q_start;
for i = 2:params.num_waypoints
    [current_q, solnInfo] = ik('panda_hand_tcp',T(:,:,i),weights,previous_q);
    if ~strcmp(solnInfo.Status, "success") && (solnInfo.PoseErrorNorm > 1e-4)
        disp("Could not solve ik for next waypoint.")
        q = [];
        con_q=[]; 
        con_t=[];
        T=[]; 
        valid=false;
        return
    end
    wpts(:, i) = current_q;

    % Check that waypoints do not have big jumps
    DIFF_THRESHOLD = 0.5;
    if max(abs(current_q-previous_q)) > DIFF_THRESHOLD
        disp("Rejected due to large jump in waypoints.")
        q = [];
        con_q=[]; 
        con_t=[];
        T=[]; 
        valid=false;
        return
    end

    previous_q = current_q;
end


% diff(wpts, 1, 2)
% Check that waypoint respects joint limits / environment&self collisions
for i = 1:size(wpts,2)
    if ~all(custom_check_valid_state(robot_ec, robot_sc, env, wpts(:,i)', jointLimits))
%         disp('Waypoints not valid.')
        q = [];
        con_q=[]; 
        con_t=[];
        T=[]; 
        valid=false;
        return
    end
end


%% Convert waypoints into trajectory
aFactor = 0.9;
[q, valid] = cartesian_wpts_to_traj(wpts, aFactor, robot_ec, robot_sc, env, jointLimits, params);
end

