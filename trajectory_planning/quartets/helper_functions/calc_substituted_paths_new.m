function [new_traj,new_path] = calc_substituted_paths( cell_inter_to_inter_70,cell_inter_to_inter_path, target_min, target_max)
%CALC_SUBSTITUTED_PATHS Substitute short for multi-segment to prevent same motions from being obvious


num_positions = size(cell_inter_to_inter_path,1);
new_traj = cell(num_positions, num_positions);
new_path = cell(num_positions, num_positions);


seq_lengths_cell = cell(num_positions, num_positions);

for r = 1:num_positions
    for c = r:num_positions

        if r ==c
            continue
        end


        seq_lengths = zeros(num_positions,1);
        possible_seq = repmat([r,0,c],num_positions,1);
        possible_seq(:,2) = 1:num_positions;

        for seq_num =  1:num_positions

            if (r==seq_num) || (c==seq_num)
                seq_lengths(seq_num) = NaN;
                continue
            end

            seq = possible_seq(seq_num,:);
            seq_length = 0;
            seq_path = [];
            seq_traj = [];
            for idx_a = 1:(numel(seq)-1)
                idx_b= idx_a+1;

                seq_a = seq(idx_a);
                seq_b = seq(idx_b);

                sub_path = cell_inter_to_inter_path{seq_a, seq_b};
                if idx_a == 1
                    seq_path = [seq_path; sub_path];
                else
                    seq_path = [seq_path;sub_path(2:end,:)];
                end

                sub_traj = cell_inter_to_inter_70{seq_a, seq_b};
                seq_traj = [seq_traj;sub_traj];

                %                 sub_length = size(sub_traj,1);
                seq_length = size(seq_traj,1);

            end

            % Skip if intermediate position is same
            seq_lengths(seq_num) = seq_length;


        end
        seq_lengths_cell{r,c} = seq_lengths;
    end
end


% 
% %% Identify highest minimum sequence length
% highest_min_seq_length = 0;
% for r=1:num_positions
%     for c = r:num_positions
% 
%         if r==c
%             continue
%         end
% 
%         seq_lengths = seq_lengths_cell{r,c};
% 
%         % Test if this min is greater than our global min
%         min_seq_length = min(seq_lengths);
%         if min_seq_length>highest_min_seq_length
%             highest_min_seq_length = min_seq_length;
%             disp([r,c, highest_min_seq_length])
% 
%         end
% 
% 
%     end
% end
% 
% 
% %% Identify lower bound
% lower_bound = Inf;
% for r=1:num_positions
%     for c = r:num_positions
% 
%         if r==c
%             continue
%         end
% 
%         seq_lengths = seq_lengths_cell{r,c};
%         seq_lengths = seq_lengths(seq_lengths<=highest_min_seq_length);
% 
%         % Test if this min is greater than our global min
%         min_seq_length = max(seq_lengths);
%         if min_seq_length<lower_bound
%             lower_bound = min_seq_length;
% 
%         end
% 
% 
%     end
% end

lower_bound = target_min;
highest_min_seq_length=target_max;
num_intervals = 4;
interval_borders = linspace(lower_bound, highest_min_seq_length, num_intervals+1);
interval_counts_diff = zeros(num_intervals,1);
interval_counts_same = zeros(num_intervals,1);

%% Choose the intermediate values falling within this range
intermediate_arr = zeros(num_positions, num_positions);
for r=1:num_positions
    for c = r:num_positions

        if r==c
            continue
        end

        seq_lengths = seq_lengths_cell{r,c};

        if (mod(r,2)==1) && (c==r+1)
            interval_counts = interval_counts_same;
        else
            interval_counts = interval_counts_diff;
        end
        
        % Sort into intervals based on priority
        [~, interval_idx_ascending] = sort(interval_counts);

        for i = 1:num_intervals

            % Define interval min/max
            interval_idx = interval_idx_ascending(i);
            interval_min = interval_borders(interval_idx);
            interval_max = interval_borders(interval_idx+1);

            % Test if there is a sequence in this interval
            mask = (seq_lengths>=interval_min).*(seq_lengths<=interval_max);

            idx = find(mask);
            if numel(idx)>=1
                break
            end
        end

        if (mod(r,2)==1) && (c==r+1)
            interval_counts_same(interval_idx) = interval_counts_diff(interval_idx)+1;
        else
            interval_counts_diff(interval_idx) = interval_counts_diff(interval_idx)+1;
        end

        idx = idx(randi(numel(idx)));  % Randomly select
        intermediate_arr(r,c) = idx;



    end
end

%% Convert to full trajectory and paths

for r = 1:num_positions
    for c = r:num_positions

        if r==c
            continue
        end

        seq = [r,intermediate_arr(r,c),c];

        % Gather path and trajectory
        seq_length = 0;
        seq_path = [];
        seq_traj = [];
        for idx_a = 1:(numel(seq)-1)
            idx_b= idx_a+1;

            seq_a = seq(idx_a);
            seq_b = seq(idx_b);

            sub_path = cell_inter_to_inter_path{seq_a, seq_b};
            if idx_a == 1
                seq_path = [seq_path; sub_path];
            else
                seq_path = [seq_path;sub_path(2:end,:)];
            end

            traj = cell_inter_to_inter_70{seq_a, seq_b};
            seq_traj = [seq_traj;traj];

            sub_length = size(traj,1);
            seq_length = seq_length + sub_length;
        end

        % Insert new path
        new_path{r,c} = seq_path;
        new_path{c,r} = flip(seq_path,1);

        % Insert new traj
        new_traj{r,c} = seq_traj;
        new_traj{c,r} = flip(seq_traj,1);


    end
end




