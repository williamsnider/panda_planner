function writeCSV_from_paths_struct(paths_struct,SAVE_DIR , params)

speed_factor = num2str(params.vScale*100);


fieldNamesList = fieldnames(paths_struct);
for f_num = 1:numel(fieldNamesList)
    f_name = fieldNamesList{f_num};

    traj = getfield(paths_struct, f_name);
    traj_name = f_name;

    writematrix(traj(:, 1:7), SAVE_DIR +traj_name+"_"+speed_factor+'%.csv')
end
end

