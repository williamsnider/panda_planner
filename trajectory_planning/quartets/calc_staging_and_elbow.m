function [q_staging,elbow_staging_to_extreme, sign_staging_to_extreme, elbow_LUT] = calc_staging_and_elbow(panda_sc_orig, ik_orig, q_extreme, T_extreme, T_staging)
%CALC_STAGING_AND_ELBOW Summary of this function goes here
%   Detailed explanation goes here

% Check that T_extreme and q_extreme are accurate
T_extreme_test = getTransform(panda_sc_orig, q_extreme, 'panda_hand_tcp');
assert(sum(sum(abs(T_extreme-T_extreme_test)))<0.0001);

% Check that T_extreme and T_staging differ only by Z-values
T_staging_test = T_staging;
T_staging_test(3,4) = 0;
T_extreme_test(3,4) = 0;
assert(sum(sum(abs(T_staging_test-T_extreme_test)))<0.0001);

% Calculate z_values from extreme to staging
step_size = 0.0001; %m
z_values = T_extreme(3,4):-step_size:T_staging(3,4);
assert(numel(z_values)==1001)

q_step_arr = zeros(numel(z_values), 9);
q_step_arr(1,:) = q_extreme;


% Interpolate from q_extreme to q_staging

for step_num = 2:numel(z_values)
    initial_guess = q_step_arr(step_num-1,:);

    T_step = T_extreme;
    T_step(3,4) = z_values(step_num);
    [q_step,solnInfo] = ik_orig('panda_hand_tcp',T_step,[1 1 1 1 1 1],initial_guess);

    assert(strcmp(solnInfo.Status, "success"))
    assert(~is_robot_in_self_collision_ignore_pairs(panda_sc_orig, q_step))
    q_step_arr(step_num,:) = q_step;
end

% Extract third joint (elbow) and fourth (sign), and flip so it goes from
% staging to extreme;
elbow_staging_to_extreme = flip(q_step_arr(:,3),1);
sign_staging_to_extreme = flip(sign(q_step_arr(:,4)),1);
q_staging = q_step_arr(end,:);

% Construct elbow LUT
elbow_LUT = zeros(numel(z_values)+1, 7);
elbow_LUT(1,:) = q_staging(1:7);
elbow_LUT(2:end, 1) = 0:(numel(z_values)-1);
elbow_LUT(2:end, 2) = flip(z_values);
elbow_LUT(2:end, 3) = elbow_staging_to_extreme;
elbow_LUT(2:end, 4) = sign_staging_to_extreme;
end

