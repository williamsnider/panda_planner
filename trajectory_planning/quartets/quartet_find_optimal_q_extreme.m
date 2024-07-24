%% Clear and load variables
run quartet_common.m

savedir = strcat(params.CustomParametersDir,'/trajectory_planning/quartets/trajectories/');
mkdir(savedir)
prefix = "20240722";

%% Select XYZ quartet_find_optimal_XYZ
XYZ = [-0.67, 0.06, 0.6477];  % Position of base of shape in qA

% Perform IK
qA_extreme_arr = [];
qW_extreme_arr = [];

count = 0;
for body_num = 1:numel(body_names)
    body_name = body_names{body_num};
    for theta_num = 1:numel(theta_list)
        theta = theta_list(theta_num);

        % Calculate correct TA and TW
        [TA, TW] = calc_TA_TW(XYZ, theta, W_SHIFT);

        disp(count)
        count = count+1;

        % Do IK for 100 valid q's
        num_loops = 100;
        qA_loop = [];
        qW_loop = [];
        for loop = 1:num_loops
            guessA = randomConfiguration(panda_sc_A);
            guessA(8:9)=0.01;
            guessW = randomConfiguration(panda_sc_W);
            guessW(8:9)=0.01;
            [qA, solnInfoA] = find_q_from_T(panda_sc_A,body_name, TA, ik_A, guessA);
            [qW, solnInfoW] = find_q_from_T(panda_sc_W,body_name, TW, ik_W, guessW);

            if strcmp(solnInfoA.Status, "success")
                qA_loop = [qA_loop;qA];
            end
            if strcmp(solnInfoW.Status, "success")
                qW_loop = [qW_loop;qW];
            end

        end

        % Choose two q's that are furthest apart on limb 1
        [submin, minidx] = min(qA_loop(:,1));
        [submax, maxidx] = max(qA_loop(:,1));
        qAa = qA_loop(minidx,:);
        qAb = qA_loop(maxidx,:);
        %
        [submin, minidx] = min(qW_loop(:,1));
        [submax, maxidx] = max(qW_loop(:,1));
        qWa = qW_loop(minidx,:);
        qWb = qW_loop(maxidx,:);


        qA_extreme_arr = [qA_extreme_arr;qAa;qAb];
        qW_extreme_arr = [qW_extreme_arr;qWa;qWb];

    end
end

% Sanity check:
plotJointMotion(panda_sc_A, qA_extreme_arr(1,:), env, params); hold on;
show(panda_sc_A, qA_extreme_arr(1,:),'collisions','on')
show(panda_sc_W, qW_extreme_arr(1,:))

num_positions = size(qA_extreme_arr,1);

%% Calculate staging positions and elbow
Z_SHIFT_EXTREME_TO_STAGING = -0.1;


A_struct = struct;
A_struct.staging_letters = ["A","B","C","D"];
A_struct.extreme_arr = qA_extreme_arr;
A_struct.inter_shift = zeros(4);
A_struct.inter_shift(1,4) = 0.05;  % Toward robot base
A_struct.inter_shift(3,4) = -0.05;  % Toward floor

extreme_arr = A_struct.extreme_arr;
staging_arr = zeros(size(extreme_arr));
elbow_staging_to_extreme_cell = cell(num_positions,1);
sign_staging_to_extreme_cell = cell(num_positions,1);
elbow_LUT_cell = cell(num_positions,1);
for i = 1:size(extreme_arr,1)
    disp(i)
    q_extreme = extreme_arr(i,:);
    [q_staging,elbow_staging_to_extreme, sign_staging_to_extreme, elbow_LUT] = calc_staging_and_elbow(panda_sc_orig, ik_orig, q_extreme, Z_SHIFT_EXTREME_TO_STAGING);
    staging_arr(i,:) = q_staging;
    elbow_staging_to_extreme_cell{i} = elbow_staging_to_extreme;
    sign_staging_to_extreme_cell{i} = sign_staging_to_extreme;
    elbow_LUT_cell{i} = elbow_LUT;
end

A_struct.staging_arr = staging_arr;
A_struct.elbow_staging_to_extreme_cell = elbow_staging_to_extreme_cell;
A_struct.sign_staging_to_extreme_cell = sign_staging_to_extreme_cell;
A_struct.elbow_LUT_cell = elbow_LUT_cell;


plotJointMotion(panda_sc_orig, qW_extreme_arr(i,:), env, params); hold on;
show(panda_sc_orig, qW_extreme_arr(i,:), "collisions", "on")


%% Make elbow LUT


%% Calculate trajectories between staging and inter positions
[cell_staging_to_inter_70, cell_staging_to_inter_path, cell_inter_to_inter_70, cell_inter_to_inter_path] = calc_between_staging_paths(A_struct.staging_arr,A_struct.inter_shift,panda_ec_orig, panda_sc_orig, ik_orig, env, params);

%% Substitute short paths
% Set random stream for reproducibility
seedval = 123;
stream = RandStream('mt19937ar', 'Seed', seedval);
RandStream.setGlobalStream(stream);

target_min = 3000;
target_max = 4500;
[cell_inter_to_inter_sub_70,cell_inter_to_inter_sub_path] = calc_substituted_paths_new(cell_inter_to_inter_70,cell_inter_to_inter_path, target_min, target_max);
plot_histogram_of_traj_lengths(cell_inter_to_inter_sub_70);


A_struct.cell_staging_to_inter_70 = cell_staging_to_inter_70;
A_struct.cell_staging_to_inter_path = cell_staging_to_inter_path;
A_struct.cell_inter_to_inter_70 = cell_inter_to_inter_70;
A_struct.cell_inter_to_inter_path = cell_inter_to_inter_path;
A_struct.cell_inter_to_inter_sub_70 = cell_inter_to_inter_sub_70;
A_struct.cell_inter_to_inter_sub_path = cell_inter_to_inter_sub_path;
A_struct.names = generate_staging_names(A_struct.staging_letters);
A_struct.mat_dir = savedir;
A_struct.traj_dir = strcat(savedir, A_struct.staging_letters(1), "/trajectories/");
A_struct.elbow_dir = strcat(savedir, A_struct.staging_letters(1), "/elbow_LUTs/");
A_struct.prefix = prefix;
mkdir(A_struct.mat_dir);
mkdir(A_struct.traj_dir);
mkdir(A_struct.elbow_dir);
convert_check_calculate_save(A_struct, panda_sc_orig, params)


%% Inputs for qA vs qW

% Position names
staging_letters = ["W","X","Y","Z"];

% q array to use
varName = 'qW';
eval(['q_arr = ',varName,'_arr;'])
inter_shift = zeros(4);
inter_shift(1,4) = 0.05;  % Positive x direction, towards base


% % Position names
% % staging_letters = 
% staging_letters = ["A","B","C","D"]; % ["W","X","Y","Z"];
% 
% % q array to use
% varName = 'qA';
% eval(['q_arr = ',varName,'_arr;'])
% inter_shift = zeros(4);
% inter_shift(1,4) = 0.05;  % Positive x direction, towards base
% inter_shift(3,4) = -0.05; % Down, towards floor

% Where to save


% Seed rng for substituting short paths for reproducibility

%% Aggregate names of positions


%% Calculate staging_to_inter and inter_to_inter paths/trajectories
[cell_staging_to_inter_70, cell_staging_to_inter_path, cell_inter_to_inter_70, cell_inter_to_inter_path] = calc_between_staging_paths(q_arr,inter_shift,panda_ec_orig, panda_sc_orig, ik_orig, env, params);




%% Substitute short paths - find seedval that has similar distributions of same and different motions
seedval_sub_short_paths = 123; % qW
target_min = 2250;
target_max = 4000;
% seedval_sub_short_paths = 127; % qA
% target_min = 2000;
% target_max = 4000;
[cell_inter_to_inter_sub_70,cell_inter_to_inter_sub_path] = calc_substituted_paths(seedval_sub_short_paths, cell_inter_to_inter_70,cell_inter_to_inter_path, target_min, target_max);

plot_histogram_of_traj_lengths(cell_inter_to_inter_sub_70);
waitforbuttonpress();



