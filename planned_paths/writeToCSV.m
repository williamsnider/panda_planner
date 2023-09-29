function writeToCSV(savename, params, paths_struct)

%% Write to csv
speed_factor = num2str(params.vScale*100);

% Joint positions
all_paths = fieldnames(paths_struct);

for path_num = 1:numel(all_paths)
    path_name = all_paths{path_num};

    % Skip path names that are just waypoints
    if contains(path_name, "wpts")
        continue
    end

    traj = getfield(paths_struct, path_name);

    writematrix(traj(:, 1:7), savename+'_'+path_name+'%.csv')
end

end

