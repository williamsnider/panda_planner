function [q_staging,elbow_staging_to_extreme, sign_staging_to_extreme, elbow_LUT] = calc_staging_and_elbow(panda_sc_orig, ik_orig, q_extreme, Z_SHIFT_EXTREME_TO_STAGING)
%CALC_STAGING_AND_ELBOW Summary of this function goes here
%   Detailed explanation goes here
T_extreme = getTransform(panda_sc_orig, q_extreme, 'panda_hand_tcp');
step_size = 0.0001; %m
z_values = T_extreme(3,4):-step_size:(T_extreme(3,4)+Z_SHIFT_EXTREME_TO_STAGING);

q_step_arr = zeros(numel(z_values), 9);
q_step_arr(1,:) = q_extreme;

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

