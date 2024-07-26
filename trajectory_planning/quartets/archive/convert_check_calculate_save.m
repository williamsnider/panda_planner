function convert_check_calculate_save(data_struct, panda_sc_orig, params)
%CONVERT_CHECK_CALCULATE_SAVE Summary of this function goes here

% Gather data from struct
cell_inter_to_inter_path = data_struct.cell_inter_to_inter_path;
cell_staging_to_inter_70 = data_struct.cell_staging_to_inter_70;
cell_inter_to_inter_sub_70 = data_struct.cell_inter_to_inter_sub_70;
cell_staging_to_inter_path = data_struct.cell_staging_to_inter_path;
cell_inter_to_inter_sub_path = data_struct.cell_inter_to_inter_sub_path;
staging_arr = data_struct.staging_arr;
names = data_struct.names;
mat_dir = data_struct.mat_dir;
traj_dir = data_struct.traj_dir;
elbow_dir = data_struct.elbow_dir;
prefix = data_struct.prefix;
staging_letters = data_struct.staging_letters;

%% Convert to full path - piece together staging_to_inter, inter_to_inter, and inter_to_staging
num_positions = size(cell_inter_to_inter_path, 1);
cell_full_70 = cell(num_positions, num_positions);
cell_full_path = cell(num_positions, num_positions);
for r=1:num_positions
    for c = 1:num_positions

        if r==c
            continue
        end

        % Combine
        full_traj = [cell_staging_to_inter_70{r}; cell_inter_to_inter_sub_70{r,c}; flip(cell_staging_to_inter_70{c},1)];
        cell_full_70{r,c} = full_traj;

        full_path = [cell_staging_to_inter_path{r}; cell_inter_to_inter_sub_path{r,c}; flip(cell_staging_to_inter_path{c},1)];
        cell_full_path{r,c} = full_path;

    end
end


%% Check full trajectories
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

        % Check no self collisions
        assert(~checkTrajForSelfCollisions(panda_sc_orig, traj, params));
    end
end

% Calculate trajectories for 10% and 40% speed
cell_full_10 = cell(size(cell_full_70));
cell_full_40 = cell(size(cell_full_70));
cell_full_70 = cell(size(cell_full_70));
% for r = 1:num_positions
%     for c=1:num_positions
%         if c<=r
%             continue
%         end
% 
%         disp([r,c])
%         
%         % Load planned path
%         planned_path = cell_full_path{r,c};
% 
%         % Convert path to trajectory
%         [traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(planned_path, panda_sc_orig,params);
%     
%         % Assign to cell
%         cell_full_10{r,c} = traj_10;
%         cell_full_40{r,c} = traj_40;
%         cell_full_70{r,c} = traj_70;
%         cell_full_10{c,r} = traj_10_reverse;
%         cell_full_40{c,r} = traj_40_reverse;
%         cell_full_70{c,r} = traj_70_reverse;
%         
% 
%         % Write to CSV - forward
%         r_name = names{r};
%         c_name = names{c};
%         fname_base = strcat(traj_dir, prefix, "_staging", r_name,"_to_","staging",c_name);
%         write_to_CSV_10_40_70(traj_10, traj_40, traj_70, fname_base);
%         
%         % Write to CSV - reverse
%         fname_base = strcat(traj_dir, prefix, "_staging", c_name,"_to_","staging",r_name);
%         write_to_CSV_10_40_70(traj_10_reverse, traj_40_reverse, traj_70_reverse, fname_base);
%     end
% end

%% Assign to struct
data_struct.cell_full_10 = cell_full_10;
data_struct.cell_full_40 = cell_full_40;
data_struct.cell_full_70 = cell_full_70;
data_struct.cell_full_path = cell_full_path;

%% Write out elbow table
for i = 1:num_positions
    name = names{i};
    fname = strcat(elbow_dir, prefix, "_staging", name,"_elbow_LUT.csv");
    elbow_LUT = data_struct.elbow_LUT_cell{i};
    
    % Sanity check
    elbow_q = elbow_LUT(1,:);
    assert(all(elbow_q==data_struct.staging_arr(i,1:7)));

    writematrix(elbow_LUT, fname)
end


%% Save struct

% Save the struct to a single .mat file
save(strcat(mat_dir, prefix,"_",staging_letters(1),".mat"), "data_struct");


end

