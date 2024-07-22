function  plot_histogram_of_traj_lengths(new_traj)
num_positions = size(new_traj,1);
traj_lengths = zeros(size(new_traj));
for r = 1:num_positions
    for c = 1:num_positions
        traj_lengths(r,c) = size(new_traj{r,c},1);
    end
end

histogram(traj_lengths(:), 'BinEdges', 250:250:5000, 'FaceColor', 'blue');
hold on;
traj_lengths_same = diag(traj_lengths,1);
traj_lengths_same = traj_lengths_same(1:2:end);
histogram(traj_lengths_same, 'BinEdges', 250:250:5000, 'FaceColor', 'red');
end

