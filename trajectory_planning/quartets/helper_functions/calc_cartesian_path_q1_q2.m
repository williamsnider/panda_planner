function [q_arr, x_vals] = calc_cartesian_path_q1_q2(q1, q2, T1, T2, ik, robot, body_name, num_steps)
%CALC_CARTESIAN_PATH_Q1_Q2 Summary of this function goes here
%   Detailed explanation goes here

%% Check inputs

% Check that q1 results in T1
T1_test = getTransform(robot, q1, body_name);
assert(sum(sum(abs(T1-T1_test)))<0.000001);

% Check that q2 results in T2
T2_test = getTransform(robot, q2, body_name);
assert(sum(sum(abs(T2-T2_test)))<0.000001);



%% Interpolate between q1 and q2
q_interp = zeros(num_steps,9);
for i = 1:9
    q_interp(:,i) = linspace(q1(i), q2(i), num_steps);
end


%% Do IK for each step, using interpolated position as initial_guess
x_vals = linspace(T1(1,4),T2(1,4),num_steps);
q_arr = zeros(size(q_interp));
q_arr(1,:) = q1;
for i = 2:num_steps

    T_step = T1;
    T_step(1,4) = x_vals(i);

    initial_guess = q_interp(i,:);

    [q_step, solnInfo] = ik(body_name, T_step, [1 1 1 1 1 1], initial_guess);
    assert(strcmp(solnInfo.Status,'success'));
    q_arr(i,:) = q_step;
end


%% Check no major jumps
assert(max(max(abs(diff(q_arr))))<0.01);
end

