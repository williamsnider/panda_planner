%% Clear and load variables




%% Clear all variables and loam parameters
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

%% Load quartet-common variables
[panda_ec_orig, panda_sc_orig, panda_ec_A, panda_sc_A, panda_ec_W, panda_sc_W, ik_orig, ik_A, ik_W, env, body_names, theta_list, W_SHIFT,Z_SHIFT_EXTREME_TO_STAGING] = load_quartet_common_variables(params);

%% Set RNG for reproducibilitiy
seedval = 123;
stream = RandStream('mt19937ar', 'Seed', seedval);
RandStream.setGlobalStream(stream);


%% Create combinations of body_names and thetas (4 shapes at 4 orientations)
body_theta_combs = {};
for body_num = 1:numel(body_names)
    for theta_num = 1:numel(theta_list)

        body_name = body_names{body_num};
        theta = theta_list(theta_num);
        body_theta_combs{end+1} = {body_name, theta};
    end
end

%% Generate q_extreme, q_staging, and q_inter
XYZ = [-0.67, 0.06, 0.5969];  % Position of base of shape in qA


% A orientations: pointing up
A_extreme_cell = cell(numel(body_theta_combs),1);
A_staging_cell = cell(numel(body_theta_combs),1);
A_inter_cell = cell(numel(body_theta_combs),1);
A_elbow_LUT_cell_paired = cell(numel(body_theta_combs),1);

T_extreme_to_staging = zeros(4);
T_extreme_to_staging(3,4) = Z_SHIFT_EXTREME_TO_STAGING;
T_staging_to_inter = zeros(4);
T_staging_to_inter(1,4) = 0.05; % Toward robot base
T_staging_to_inter(3,4) = -0.05;  % Toward floor

parfor body_theta_num = 1:numel(body_theta_combs)
    warning('off', 'all');
    disp(body_theta_num)
    body_name = body_theta_combs{body_theta_num}{1};
    theta = body_theta_combs{body_theta_num}{2};
    [TA_extreme, TW_extreme] = calc_TA_TW(XYZ, theta, W_SHIFT);  % Need to shift to second output for W

    T_extreme = TA_extreme;
    panda_sc_restricted = panda_sc_A;
    ik_restricted = ik_A;

    [q_extreme, q_staging, q_inter, elbow_cell] = find_q_extreme_staging_inter(T_extreme, body_name, T_extreme_to_staging, T_staging_to_inter, panda_sc_restricted, panda_sc_orig, ik_restricted, ik_orig, env);
    A_extreme_cell{body_theta_num} = q_extreme;
    A_staging_cell{body_theta_num} = q_staging;
    A_inter_cell{body_theta_num} = q_inter;
    A_elbow_LUT_cell_paired{body_theta_num} = elbow_cell;
end

% Convert from cell to array
A_extreme_arr = [];
A_staging_arr = [];
A_inter_arr = [];
A_elbow_LUT_cell = {};
for i = 1:numel(A_extreme_cell)
    A_extreme_arr = [A_extreme_arr; A_extreme_cell{i}(1,:);A_extreme_cell{i}(2,:)];
    A_staging_arr = [A_staging_arr; A_staging_cell{i}(1,:); A_staging_cell{i}(2,:)];
    A_inter_arr = [A_inter_arr; A_inter_cell{i}(1,:); A_inter_cell{i}(2,:)];
    A_elbow_LUT_cell{end+1} = A_elbow_LUT_cell_paired{i}{1};
    A_elbow_LUT_cell{end+1} = A_elbow_LUT_cell_paired{i}{2};
end


% %% Find q_extreme, q_staging, and q_inter (two lineages)
% XYZ = [-0.67, 0.06, 0.5969];  % Position of base of shape in qA
% Z_SHIFT_EXTREME_TO_STAGING = -0.1;
% 
% 
% % Check valid ik
% % Check no self collision
% % Check no env collision (except or q_extreme)
% % Check close to previous
% 
% %% Select XYZ quartet_find_optimal_XYZ
% 
% XYZ = [-0.67, 0.06, 0.5969];  % Position of base of shape in qA
% % !!! NOTE !!! - make sure that the collision boxes in
% % build_collision_environment reflect this position and how the arm port
% % will be in the physical rig.
% 
% % Perform IK
% qA_extreme_arr = [];
% qW_extreme_arr = [];
% 
% count = 0;
% for body_num = 1:numel(body_names)
%     body_name = body_names{body_num};
%     for theta_num = 1:numel(theta_list)
%         theta = theta_list(theta_num);
% 
%         % Calculate correct TA and TW
%         [TA, TW] = calc_TA_TW(XYZ, theta, W_SHIFT);
% 
%         disp(count)
%         count = count+1;
% 
%         % Do IK for 100 valid q's
%         num_loops = 100;
%         qA_loop = [];
%         qW_loop = [];
%         for loop = 1:num_loops
%             guessA = randomConfiguration(panda_sc_A);
%             guessA(8:9)=0.01;
%             guessW = randomConfiguration(panda_sc_W);
%             guessW(8:9)=0.01;
%             [qA, solnInfoA] = find_q_from_T(panda_sc_A,body_name, TA, ik_A, guessA);
%             [qW, solnInfoW] = find_q_from_T(panda_sc_W,body_name, TW, ik_W, guessW);
% 
%             if strcmp(solnInfoA.Status, "success")
%                 qA_loop = [qA_loop;qA];
%             end
%             if strcmp(solnInfoW.Status, "success")
%                 qW_loop = [qW_loop;qW];
%             end
% 
%         end
% 
%         % Choose two q's that are furthest apart on limb 1
%         [submin, minidx] = min(qA_loop(:,1));
%         [submax, maxidx] = max(qA_loop(:,1));
%         qAa = qA_loop(minidx,:);
%         qAb = qA_loop(maxidx,:);
%         %
%         [submin, minidx] = min(qW_loop(:,1));
%         [submax, maxidx] = max(qW_loop(:,1));
%         qWa = qW_loop(minidx,:);
%         qWb = qW_loop(maxidx,:);
% 
% 
%         qA_extreme_arr = [qA_extreme_arr;qAa;qAb];
%         qW_extreme_arr = [qW_extreme_arr;qWa;qWb];
% 
%     end
% end
% 
% % % Sanity check:
% % plotJointMotion(panda_sc_A, qA_extreme_arr(3,:), env, params); hold on;
% % show(panda_sc_W, qW_extreme_arr(3,:))

num_positions = size(A_extreme_arr,1);




%% Calculate staging positions and elbow
savedir = strcat(params.CustomParametersDir,'/trajectory_planning/quartets/paths/');
mkdir(savedir)
prefix = "20240725";
Z_SHIFT_EXTREME_TO_STAGING = -0.1;


%% A orientations
A_struct = struct;
A_struct.staging_letters = ["A","B","C","D"];
A_struct.extreme_arr = A_extreme_arr;
A_struct.staging_arr = A_staging_arr;
A_struct.inter_arr = A_inter_arr;
% A_struct.inter_shift = zeros(4);
% A_struct.inter_shift(1,4) = 0.05;  % Toward robot base
% A_struct.inter_shift(3,4) = -0.05;  % Toward floor
A_struct.target_path_length_min = 3000;
A_struct.target_path_length_max = 4500;
A_struct.mat_dir = savedir;
A_struct.traj_dir = strcat(savedir, A_struct.staging_letters(1), "/trajectories/");
A_struct.elbow_dir = strcat(savedir, A_struct.staging_letters(1), "/elbow_LUTs/");
A_struct.prefix = prefix;
A_struct.staging_arr = A_staging_arr;
A_struct.extreme_arr = A_extreme_arr;
A_struct.inter_arr = A_inter_arr;
% A_struct.elbow_staging_to_extreme_cell = A_elbow_LUT_cell_paired;
% A_struct.sign_staging_to_extreme_cell = A_sign_staging_to_extreme_cell;
A_struct.elbow_LUT_cell = A_elbow_LUT_cell;

A_struct = quartet_find_staging_and_trajectories(A_struct, panda_ec_orig, panda_sc_orig, ik_orig, env, Z_SHIFT_EXTREME_TO_STAGING, params);



% 
% 
% %% Inputs for qA vs qW
% 
% % Position names
% staging_letters = ["W","X","Y","Z"];
% 
% % q array to use
% varName = 'qW';
% eval(['q_arr = ',varName,'_arr;'])
% inter_shift = zeros(4);
% inter_shift(1,4) = 0.05;  % Positive x direction, towards base
% 
% 
% % % Position names
% % % staging_letters = 
% % staging_letters = ["A","B","C","D"]; % ["W","X","Y","Z"];
% % 
% % % q array to use
% % varName = 'qA';
% % eval(['q_arr = ',varName,'_arr;'])
% % inter_shift = zeros(4);
% % inter_shift(1,4) = 0.05;  % Positive x direction, towards base
% % inter_shift(3,4) = -0.05; % Down, towards floor
% 
% % Where to save
% 
% 
% % Seed rng for substituting short paths for reproducibility
% 
% %% Aggregate names of positions
% 
% 
% %% Calculate staging_to_inter and inter_to_inter paths/trajectories
% [cell_staging_to_inter_70, cell_staging_to_inter_path, cell_inter_to_inter_70, cell_inter_to_inter_path] = calc_between_staging_paths(q_arr,inter_shift,panda_ec_orig, panda_sc_orig, ik_orig, env, params);
% 
% 
% 
% 
% %% Substitute short paths - find seedval that has similar distributions of same and different motions
% seedval_sub_short_paths = 123; % qW
% target_min = 2250;
% target_max = 4000;
% % seedval_sub_short_paths = 127; % qA
% % target_min = 2000;
% % target_max = 4000;
% [cell_inter_to_inter_sub_70,cell_inter_to_inter_sub_path] = calc_substituted_paths(seedval_sub_short_paths, cell_inter_to_inter_70,cell_inter_to_inter_path, target_min, target_max);
% 
% plot_histogram_of_traj_lengths(cell_inter_to_inter_sub_70);
% waitforbuttonpress();
% 
% 
% 
