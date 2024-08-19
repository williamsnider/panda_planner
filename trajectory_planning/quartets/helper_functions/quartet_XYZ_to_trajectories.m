function data_struct = quartet_XYZ_to_trajectories(data_struct, env, params)
    
    %% Set RNG for reproducibility
    seedval = 123;
    stream = RandStream('mt19937ar', 'Seed', seedval);
    RandStream.setGlobalStream(stream);

    %% Collect variables from data_struct
    calc_T = data_struct.calc_T;
    body_names = data_struct.body_names;
    theta_list = data_struct.theta_list;
    XYZ = data_struct.XYZ;
    T_extreme_to_staging = data_struct.T_extreme_to_staging;
    T_staging_to_inter = data_struct.T_staging_to_inter;
    panda_sc_restricted = data_struct.panda_sc_restricted;
    panda_sc_orig = data_struct.panda_sc_orig;
    ik_restricted = data_struct.ik_restricted;
    ik_orig = data_struct.ik_orig;



    %% Create combinations of body_names and thetas (4 shapes at 4 orientations)
    body_theta_combs = {};
    for body_num = 1:numel(body_names)
        for theta_num = 1:numel(theta_list)
            body_name = body_names{body_num};
            theta = theta_list(theta_num);
            T_extreme = calc_T(XYZ, theta); 
            body_theta_combs{end+1} = {body_name, theta, T_extreme};
        end
    end

    %% Generate q_extreme, q_staging, q_inter, and elbow_LUT
    % A orientations: pointing up
    A_extreme_cell = cell(numel(body_theta_combs),1);
    A_staging_cell = cell(numel(body_theta_combs),1);
    A_inter_cell = cell(numel(body_theta_combs),1);
    A_elbow_LUT_cell_paired = cell(numel(body_theta_combs),1);

    for body_theta_num = 1:numel(body_theta_combs)
        warning('off', 'all');
        disp(body_theta_num)
        body_name = body_theta_combs{body_theta_num}{1};
        theta = body_theta_combs{body_theta_num}{2};
        T_extreme = body_theta_combs{body_theta_num}{3};
        SV = data_struct.SV;

        [q_extreme, q_staging, q_inter, elbow_cell] = find_q_extreme_staging_inter(SV, T_extreme, body_name, T_extreme_to_staging, T_staging_to_inter, panda_sc_restricted, panda_sc_orig, ik_restricted, ik_orig, env,params);
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
        A_extreme_arr = [A_extreme_arr; A_extreme_cell{i}(1,:); A_extreme_cell{i}(2,:)];
        A_staging_arr = [A_staging_arr; A_staging_cell{i}(1,:); A_staging_cell{i}(2,:)];
        A_inter_arr = [A_inter_arr; A_inter_cell{i}(1,:); A_inter_cell{i}(2,:)];
        A_elbow_LUT_cell{end+1} = A_elbow_LUT_cell_paired{i}{1};
        A_elbow_LUT_cell{end+1} = A_elbow_LUT_cell_paired{i}{2};
    end
    num_positions = size(A_extreme_arr,1);

    %% Assign values to struct
    data_struct.staging_arr = A_staging_arr;
    data_struct.extreme_arr = A_extreme_arr;
    data_struct.inter_arr = A_inter_arr;
    data_struct.elbow_LUT_cell = A_elbow_LUT_cell;
end

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
