function [new_traj,new_path] = calc_substituted_paths(seedval, cell_inter_to_inter_70,cell_inter_to_inter_path, target_min, target_max)
%CALC_SUBSTITUTED_PATHS Substitute short for multi-segment to prevent same motions from being obvious


%% 
stream = RandStream('mt19937ar', 'Seed', seedval);
RandStream.setGlobalStream(stream);

new_traj = cell_inter_to_inter_70;
new_path = cell_inter_to_inter_path;
num_positions = size(cell_inter_to_inter_path,1);



for r = 1:num_positions
    for c = r:num_positions

        if r ==c
            continue
        end


        traj = cell_inter_to_inter_70{r,c};
        traj_length = size(traj,1);

        planned_path = cell_inter_to_inter_path{r,c};
        path_num = size(planned_path,1);

        % Ensure minimum length AND at least 1 intermediate configuration
        if ((traj_length>=target_min)  && (path_num>2))
            continue
        end

        disp(strcat(num2str(r)," ", num2str(c)))



        possible_seq = [];
        for i1 = 1:32

            if (i1==r) || (i1==c)
                continue
            end

            seq = [r,i1,c];
            possible_seq = [possible_seq;seq];

        end

        shuffled_seq = possible_seq(randperm(size(possible_seq,1)),:);

        found_seq = false;
        for seq_num = 1:size(shuffled_seq)

            seq = shuffled_seq(seq_num,:);
            seq = seq(seq>0);  % Remove zero idx; shortens to just 1 intermediate

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



            if (target_min < seq_length) && (target_max > seq_length)

                % Insert new path
                new_path{r,c} = seq_path;
                new_path{c,r} = flip(seq_path,1);

                % Insert new traj
                new_traj{r,c} = seq_traj;
                new_traj{c,r} = flip(seq_traj,1);

                % Exit loop
                found_seq=true;
                break
            end


        end
        if ~found_seq
            disp(strcat("Failed for ",num2str(r)," ", num2str(c)))
        end

    end
end

traj_lengths = zeros(size(new_traj));
for r = 1:num_positions
    for c = 1:num_positions
        traj_lengths(r,c) = size(new_traj{r,c},1);
    end
end

histogram(traj_lengths(:), 'BinEdges', 100:250:5000, 'FaceColor', 'blue');
hold on;
traj_lengths_same = diag(traj_lengths,1);
traj_lengths_same = traj_lengths_same(1:2:end);
histogram(traj_lengths_same, 'BinEdges', 100:250:5000, 'FaceColor', 'red');
end

