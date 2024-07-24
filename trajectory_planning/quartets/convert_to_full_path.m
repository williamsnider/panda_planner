function [cell_full_70,cell_full_path] = convert_to_full_path(cell_inter_to_inter_path,cell_staging_to_inter_70, cell_inter_to_inter_sub_70, cell_staging_to_inter_path, cell_inter_to_inter_sub_path)
%CONVERT_TO_FULL_PATH Summary of this function goes here
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


end

