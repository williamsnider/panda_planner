function [traj,valid] = cartesian_wpts_to_traj(wpts, aFactor, robot_ec, robot_sc, env, jointLimits, params)


%% Calculate Trajectory - velocity and acceleration constraints
vellim = [-params.vMaxAll', params.vMaxAll'];
accellim = [-params.aMaxAll', params.aMaxAll']*aFactor;  % Scale further to make sure acceleration within limits


% Loop through different NumSamples to ensure velocity/acceleration correct
traj = [];
for NumSamples = 100:100:800
    valid = true;

    [con_q,con_qd,con_qdd,con_t] = contopptraj(wpts,vellim,accellim, NumSamples=NumSamples);

    if con_t(end) > 10
        disp("Warning: Cartesian path is over 10s long")
        valid=false;
        continue
    end




    % Fit spline to q given t (contopptraj t is not linearly spaced)
    end_time = ceil(con_t(end)*1000)/1000; % round up to next 0.001
    spline_t = 0:0.001:end_time;
    spline_q = spline(con_t,con_q,spline_t);  % Can now parameterize in linear steps of t
    spline_q(:,end) = con_q(:,end); % Ensure stopping position is at final waypoint

    % plot_derivatives(spline_q')
    % wpts(:,end)'-spline_q(:,end)'


    % Note - fitting a spline to the output of contopptraj can result in the
    % inputted accel/vel constraints being slightly exceeded. This is
    % acceptable, as long as later on there is a final check to ensure that the
    % absolute limits of the robot's accel/vel are not exceeded.


    % Smooth to reduce jerk (sudden changes in acceleration)
    smoothed_q = smooth_path(spline_q, params.window_size);

    % Pad edges so that jerk is 0 at start/end
    pad_size = 3;
    padded_q = [repmat(smoothed_q(:,1),1,pad_size),smoothed_q, repmat(smoothed_q(:,end),1,pad_size) ];

    % Transpose
    q = padded_q';

    % Round to 10 decimal places (avoid small errors that result in state validator identifying joint positions as beyond the joint limits)
    q = round(q, 10);
    % plot_derivatives(q);

    %% Conduct checks

    % Absolute limits of robot not exceeded
    if ~checkTrajectory(q, q(1,:), q(end,:), params)
        disp("checkTrajectory FAIL")
        valid = false;
        continue
    end

    % Check arrays equal in size
    assert(size(q,2) == size(jointLimits,2));

    % Joint limits not exceeded
    aboveMin = all(all(q - jointLimits(1,:) >= -1e-4));
    belowMax = all(all(q - jointLimits(2,:) <= 1e-4));
    if ~aboveMin || ~belowMax
        valid = false;
        disp('Rejected due to violation of joint limits.')
        continue
    end


    % % Jerk not exceeded
    % [~,~,~,smoothed_qddd] = calc_trajectory_derivatives(q);
    % jPeak = max(abs(smoothed_qddd),[],1);
    % if any(jPeak> jMaxAll)
    %     valid = false;
    % end

    % Path is linear within a threshold Sample points and see how far they are
    % from line
    DISTANCE_THRESHOLD = 0.0015;
    num_points = size(q, 1);
    % sample_interval = round(num_points/num_waypoints/2);  % Sample 2x frequency of waypoints
    indices = 1:round(params.num_waypoints/2):num_points; %
    dists = [];
    T1 = getTransform(robot_ec, q(1,:), 'panda_hand_tcp');
    v1 = T1(1:3,4);
    T2 = getTransform(robot_ec, q(end,:), 'panda_hand_tcp');
    v2 = T2(1:3,4);
    a = v1-v2;
    for i = 1:numel(indices)
        idx = indices(i);
        T3 = getTransform(robot_ec, q(idx,:), 'panda_hand_tcp');
        p1 = T3(1:3, 4);
        b = v1-p1;
        dists(end+1) = norm(cross(a,b)) / norm(a);
    end
    if any(dists>DISTANCE_THRESHOLD)
        disp('Path is nonlinear')
        valid = false;
        continue
    end

    % Final position is within joint limits
    if ~custom_check_valid_state(robot_ec, robot_sc, env, q(end,:), jointLimits)
        valid=false;
        continue
    end

    if valid==false
        disp('invalid')
        continue
    end

    traj = q;

    if valid == true
        break
    end

end

end

