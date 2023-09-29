function all_trajectory = joint_path_to_traj(planned_path, params)

%% Find optimal trajectory
all_trajectory = [];
for segment_num=1:size(planned_path,1)-1
    s0All = planned_path(segment_num,:);
    s1All = planned_path(segment_num+1,:);
    segment_trajectory = findMultipleJointProfiles(s0All, s1All, params);

    % Pad start/end with same position to smooth out multi-movement trajectories
    PADDING = params.multiMovementPadding;
    padded_trajectory = [repmat(segment_trajectory(:,1), [1, PADDING]), segment_trajectory, repmat(segment_trajectory(:,end), [1,PADDING])];
    segment_trajectory=padded_trajectory;

    % Group together trajectories
    all_trajectory = [all_trajectory, segment_trajectory];
end

% Smooth
all_trajectory = smooth_path(all_trajectory, params.smoothing_window_size);

% Pad ends
all_trajectory = [repmat(all_trajectory(:,1), [1, params.smoothing_window_size]), all_trajectory, repmat(all_trajectory(:,end), [1, params.smoothing_window_size])];


% Transpose
all_trajectory = all_trajectory';



end
