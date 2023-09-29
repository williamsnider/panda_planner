function smoothed_q = smooth_path(q,window_size)
% Smooths to reduce acceleration/jerk
q_extra = [repmat(q(:,1),1,window_size), q,repmat(q(:,end),1,window_size) ];  % Pad edges so jerk at start/end is minimal
smoothed_q = movmean(q_extra, window_size, 2, 'Endpoints','shrink');

end

