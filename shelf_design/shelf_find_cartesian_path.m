function [found_valid_cartesian_path, T, combined, all_paths] = shelf_find_cartesian_path(z,radius,theta, SHAPE_ROTATION_ABOUT_Z, panda_ec, panda_sc, ik, weights, env, ss,ABOVE_HEIGHT, OUT_DIST, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute, jointMin, jointMax )

combined = [];

% Calc pose
x = cos(theta) * radius;
y = sin(theta) * radius;
T = eul2tform([0, pi/2,-theta+SHAPE_ROTATION_ABOUT_Z]);
T(1:3, 4) = [x,y,z];

% Find cartesian path
found_valid_cartesian_path = false;
loop_count = 0;
for i=1:10
    loop_count = loop_count+1;

    initialGuess = randomConfiguration(panda_ec);
    [q,solnInfo] = ik('panda_hand_tcp',T,weights,initialGuess);

    %Redo inverse kinematics if q is outside joint limits or in collision
    q = round(q, 10);  % Rounding ensures not exceeding joint limits by precision error
    if ~custom_check_valid_state(panda_ec, panda_sc, env, q, ss.StateBounds')
        continue
    end

    % Calc poses for cartesian paths
    pose_slot = T;

    pose_above = pose_slot;
    pose_above(3,4) = pose_above(3,4)+ABOVE_HEIGHT;

    vec_norm = pose_above(1:2,3);
    pose_out = pose_above;
    pose_out(1:2, 4) = pose_out(1:2,4) - vec_norm*OUT_DIST;

    poseArray = cat(3,pose_slot, pose_above, pose_out);

    window_size = 50;  % Affects smoothing of cartesian path (for jerk)
    num_waypoints = 10;  % Affects how densely the cartesian path must be linear
    [all_paths, all_valid] = calc_sequence_cartesian_paths(panda_ec, panda_sc, env, ss.StateBounds', poseArray,q,window_size, num_waypoints, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute, jointMin, jointMax);

    if all_valid==true
        found_valid_cartesian_path = true;
        combined = [all_paths{1}; all_paths{2}];
        break;
    else
        combined = [];
        all_paths = cell(1);
    end
end
end