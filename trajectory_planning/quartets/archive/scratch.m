
%% Clear all variables and loam parameters
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

%% Load quartet-common variables
[panda_ec_orig, panda_sc_orig, panda_ec_A, panda_sc_A, panda_ec_W, panda_sc_W, ik_orig, ik_A, ik_W, env, body_names, theta_list, W_SHIFT] = load_quartet_common_variables(params);

XYZ = [-0.67, 0.06, 0.5969];
theta = theta_list(2);

[TA, TW] = calc_TA_TW(XYZ, theta, W_SHIFT);
T_extreme = TA;
T_staging = T_extreme;
T_staging(3,4) = T_staging(3,4)-0.1;

initial_guess = randomConfiguration(panda_sc_A);
initial_guess(8:9) = 0.01;

[q_extreme,solnInfo] = ik_A('panda_cylinder1',T_extreme,[1 1 1 1 1 1],initial_guess)
[q_staging, solnInfo] = ik_orig('panda_cylinder1', T_staging, [1 1 1 1 1 1], q_extreme)

% Interpolate
num_steps = 1001;
body_name = 'panda_cylinder1';
q_arr = calc_cartesian_path_q1_q2(q_extreme, q_staging, T_extreme, T_staging, ik_orig, panda_sc_orig, body_name, num_steps)

% [q_staging,elbow_staging_to_extreme, sign_staging_to_extreme, elbow_LUT] = calc_staging_and_elbow(panda_sc_orig, ik_orig, q_extreme, T_extreme, T_staging)
