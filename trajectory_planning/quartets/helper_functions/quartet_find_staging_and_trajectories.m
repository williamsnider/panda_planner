function [data_struct] = quartet_find_staging_and_trajectories(data_struct, panda_ec_orig, panda_sc_orig, ik_orig, env, Z_SHIFT_EXTREME_TO_STAGING, params)


%QUARTET_FIND_STAGING_AND_TRAJECTORIES Calculates the staging positions and
%trajectories for an orientation



mkdir(data_struct.mat_dir);
mkdir(data_struct.traj_dir);
mkdir(data_struct.elbow_dir);



%% Calculate the staging positions (downshifted) and elbow_LUT for each q_extreme

% extreme_arr = data_struct.extreme_arr;
% % staging_arr = zeros(size(extreme_arr));
% num_positions = size(extreme_arr,1);
% elbow_staging_to_extreme_cell = cell(num_positions,1);
% sign_staging_to_extreme_cell = cell(num_positions,1);
% elbow_LUT_cell = cell(num_positions,1);
% parfor i = 1:size(extreme_arr,1)
%     disp(i)
%     q_extreme = extreme_arr(i,:);
%     [q_staging,elbow_staging_to_extreme, sign_staging_to_extreme, elbow_LUT] = calc_staging_and_elbow(panda_sc_orig, ik_orig, q_extreme, Z_SHIFT_EXTREME_TO_STAGING);
%     staging_arr(i,:) = q_staging;
%     elbow_staging_to_extreme_cell{i} = elbow_staging_to_extreme;
%     sign_staging_to_extreme_cell{i} = sign_staging_to_extreme;
%     elbow_LUT_cell{i} = elbow_LUT;
% end
% 
% % Assign to data struct
% data_struct.staging_arr = staging_arr;
% data_struct.elbow_staging_to_extreme_cell = elbow_staging_to_extreme_cell;
% data_struct.sign_staging_to_extreme_cell = sign_staging_to_extreme_cell;
% data_struct.elbow_LUT_cell = elbow_LUT_cell;
% 
% % Plot results
% % plotJointMotion(panda_sc_orig, qW_extreme_arr(i,:), env, params); hold on;
% % show(panda_sc_orig, qW_extreme_arr(i,:), "collisions", "on")


%% Calculate trajectories between staging and inter positions
[cell_staging_to_inter_70, cell_staging_to_inter_path, cell_inter_to_inter_70, cell_inter_to_inter_path] = calc_between_inter_paths(data_struct.staging_arr,data_struct.inter_arr,panda_ec_orig, panda_sc_orig, env, params);

%% Substitute short paths to ensure that all the staging_to_staging trajectories are the same length

% Set random stream for reproducibility
seedval = 123;
stream = RandStream('mt19937ar', 'Seed', seedval);
RandStream.setGlobalStream(stream);

% Choose paths that have balanced lengths
[cell_inter_to_inter_sub_70,cell_inter_to_inter_sub_path] = calc_substituted_paths_new(cell_inter_to_inter_70,cell_inter_to_inter_path, data_struct.target_path_length_min, data_struct.target_path_length_max);
plot_histogram_of_traj_lengths(cell_inter_to_inter_sub_70);

% Assign results to struct
data_struct.cell_staging_to_inter_70 = cell_staging_to_inter_70;
data_struct.cell_staging_to_inter_path = cell_staging_to_inter_path;
data_struct.cell_inter_to_inter_70 = cell_inter_to_inter_70;
data_struct.cell_inter_to_inter_path = cell_inter_to_inter_path;
data_struct.cell_inter_to_inter_sub_70 = cell_inter_to_inter_sub_70;
data_struct.cell_inter_to_inter_sub_path = cell_inter_to_inter_sub_path;
data_struct.names = generate_staging_names(data_struct.staging_letters);



%% Convert to full path - piece together staging_to_inter, inter_to_inter, and inter_to_staging
num_positions = size(cell_inter_to_inter_path, 1);
cell_full_70 = cell(num_positions, num_positions);
cell_full_path = cell(num_positions, num_positions);
for r=1:num_positions
    for c = 1:num_positions

        if r>=c
            continue
        end

        % Combine
        full_traj = [cell_staging_to_inter_70{r}; cell_inter_to_inter_sub_70{r,c}; flip(cell_staging_to_inter_70{c},1)];
        cell_full_70{r,c} = full_traj;
        cell_full_70{c,r} = flip(full_traj,1);

        full_path = [cell_staging_to_inter_path{r}; cell_inter_to_inter_sub_path{r,c}; flip(cell_staging_to_inter_path{c},1)];
        cell_full_path{r,c} = full_path;
        cell_full_path{c,r} = flip(full_path,1);

    end
end

%% Shut down parpool to save RAM
poolobj = gcp('nocreate');
delete(poolobj);

%% Check full trajectories
staging_arr = data_struct.staging_arr;
parfor r = 1:num_positions
    for c=1:num_positions
        if r==c
            continue
        end

        disp([r,c])
        
        % Load traj
        traj = cell_full_70{r,c};

        % Check kinematics and start/stop
        assert(checkTrajKinematics(traj, staging_arr(r,:),  staging_arr(c,:), params))

%         % Check no self collisions
%         assert(~checkTrajForSelfCollisions(panda_sc_orig, traj, params));
    end
end

% Calculate trajectories for 10% and 40% speed
cell_full_10 = cell(size(cell_full_70));
cell_full_40 = cell(size(cell_full_70));
cell_full_70 = cell(size(cell_full_70));
names = data_struct.names;
parfor r = 1:num_positions
    for c=1:num_positions
        if c<=r
            continue
        end

        disp([r,c])
        
        % Load planned path
        planned_path = cell_full_path{r,c};

        % Convert path to trajectory
        [traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(planned_path, panda_sc_orig,params);
    
        % Assign to cell
        cell_full_10{r,c} = traj_10;
        cell_full_40{r,c} = traj_40;
        cell_full_70{r,c} = traj_70;

        % Write to CSV - forward
        r_name = names{r};
        c_name = names{c};
        fname_base = strcat(data_struct.traj_dir, data_struct.prefix, "_staging", r_name,"_to_","staging",c_name);
        write_to_CSV_10_40_70(traj_10, traj_40, traj_70, fname_base);
        
        % Write to CSV - reverse
        fname_base = strcat(data_struct.traj_dir, data_struct.prefix, "_staging", c_name,"_to_","staging",r_name);
        write_to_CSV_10_40_70(traj_10_reverse, traj_40_reverse, traj_70_reverse, fname_base);
    end
end

% Flip trajectories
for r = 1:num_positions
    for c = 1:num_positions

        if c<=r
            continue
        end
        cell_full_10{c,r} = cell_full_10{r,c};
        cell_full_40{c,r} = cell_full_40{r,c};
        cell_full_70{c,r} = cell_full_70{r,c};
    end
end


%% Assign to struct
data_struct.cell_full_10 = cell_full_10;
data_struct.cell_full_40 = cell_full_40;
data_struct.cell_full_70 = cell_full_70;
data_struct.cell_full_path = cell_full_path;

%% Write out elbow table
for i = 1:num_positions
    name = data_struct.names{i};
    fname = strcat(data_struct.elbow_dir, data_struct.prefix, "_staging", name,"_elbow_LUT.csv");
    elbow_LUT = data_struct.elbow_LUT_cell{i};
    
    % Sanity check
    elbow_q = elbow_LUT(1,:);
    assert(sum(sum(abs(elbow_q-data_struct.staging_arr(i,1:7))))<0.00001);

    writematrix(elbow_LUT, fname)
end


%% Save struct

% Save the struct to a single .mat file
save(strcat(data_struct.mat_dir, data_struct.prefix,"_",data_struct.staging_letters(1),".mat"), 'data_struct', '-v7.3'); % Specify version because file is large.
end

