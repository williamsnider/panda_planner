function  plot_histogram_of_traj_lengths(new_traj)
num_positions = size(new_traj,1);
traj_lengths = zeros(size(new_traj));
for r = 1:num_positions
    for c = 1:num_positions
        traj_lengths(r,c) = size(new_traj{r,c},1);
    end
end
traj_lengths_same = diag(traj_lengths,1);
traj_lengths = traj_lengths(traj_lengths>0);
bin_edges = linspace(min(traj_lengths(:)), max(traj_lengths(:)), 5);
histogram(traj_lengths(:), 'BinEdges', bin_edges, 'FaceColor', 'blue');
hold on;
traj_lengths_same = traj_lengths_same(1:2:end);
histogram(traj_lengths_same, 'BinEdges', bin_edges, 'FaceColor', 'red');

% Display highest length pairs
[r,c] = find(traj_lengths==max(max(traj_lengths)), 1)
disp(traj_lengths(r,c))
end

