function elbow_LUT = construct_elbow_LUT(q_extreme_to_staging_arr,x_vals_extreme_to_staging)
%CONSTRUCT_ELBOW_LUT Summary of this function goes here
%   Detailed explanation goes here

assert(numel(x_vals_extreme_to_staging)==1501);

elbow_staging_to_extreme = flip(q_extreme_to_staging_arr(:,3),1);
sign_staging_to_extreme = flip(sign(q_extreme_to_staging_arr(:,4)),1);
q_staging = q_extreme_to_staging_arr(end,:);

% Construct elbow LUT
elbow_LUT = zeros(numel(x_vals_extreme_to_staging)+1, 7);
elbow_LUT(1,:) = q_staging(1:7);
elbow_LUT(2:end, 1) = 0:(numel(x_vals_extreme_to_staging)-1);
elbow_LUT(2:end, 2) = flip(x_vals_extreme_to_staging);
elbow_LUT(2:end, 3) = elbow_staging_to_extreme;
elbow_LUT(2:end, 4) = sign_staging_to_extreme;
end

