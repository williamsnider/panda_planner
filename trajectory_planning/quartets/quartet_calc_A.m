%% Clear all variables and loam parameters
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

%% Load quartet-common variables
[panda_ec_orig, panda_sc_orig, panda_ec_A, panda_sc_A, ik_orig, ik_A, env, body_names, theta_list, X_SHIFT_EXTREME_TO_STAGING] = load_quartet_common_variables(params);

%% Create struct to pass all data into the pipeline
data_struct = struct;

%% Assign values to struct
data_struct.XYZ = [-0.77, 0.08, 0.6477];  % Position of base of shape in qA - should be identical to W orientations!
data_struct.savedir = strcat(params.CustomParametersDir,'/trajectory_planning/quartets/paths/');
data_struct.prefix = "20240819";
data_struct.X_SHIFT_EXTREME_TO_STAGING = X_SHIFT_EXTREME_TO_STAGING;
data_struct.staging_letters = ["A","B","C","D"];
data_struct.target_path_length_min = 2000;
data_struct.target_path_length_max = 4000;
data_struct.mat_dir = data_struct.savedir;
data_struct.traj_dir = strcat(data_struct.savedir, data_struct.staging_letters(1), "/trajectories/");
data_struct.elbow_dir = strcat(data_struct.savedir, data_struct.staging_letters(1), "/elbow_LUTs/");

data_struct.T_extreme_to_staging = zeros(4);
data_struct.T_extreme_to_staging(1,4) = data_struct.X_SHIFT_EXTREME_TO_STAGING;
data_struct.T_staging_to_inter = zeros(4);
data_struct.T_staging_to_inter(1,4) = 0.05; % Toward robot base
data_struct.calc_T = @calc_TA;
data_struct.panda_sc_restricted = panda_sc_A;
data_struct.panda_sc_orig = panda_sc_orig;
data_struct.panda_ec_orig = panda_ec_orig;
data_struct.ik_orig = ik_orig;
data_struct.ik_restricted = ik_A;
data_struct.body_names = body_names;
data_struct.theta_list = theta_list;
data_struct.SV = construct_state_validator(panda_ec_orig, panda_sc_orig, env, params);




%% Calculate using this struct as input
data_struct = quartet_XYZ_to_trajectories(data_struct, env, params);
 
%% Convert XYZ_to_trajectories
data_struct = quartet_find_staging_and_trajectories(data_struct, panda_ec_orig, panda_sc_orig, ik_orig, env, X_SHIFT_EXTREME_TO_STAGING, params);
