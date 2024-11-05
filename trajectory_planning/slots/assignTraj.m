function paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale)

% Convert vScale to string
vString = num2str(round(100*vScale));

paths_struct.("slot_to_above_" + vString) = all_paths{1}(:,1:7);
paths_struct.("above_to_slot_" + vString) = flip(paths_struct.("slot_to_above_" + vString), 1);
paths_struct.("wpts_slot_to_above_" + vString) = all_wpts(1:9, :);
paths_struct.("above_to_out_" + vString) = all_paths{2}(:,1:7);
paths_struct.("out_to_above_" + vString) = flip(paths_struct.("above_to_out_" + vString), 1);
paths_struct.("wpts_above_to_out_" + vString) = all_wpts(10:18, :);
paths_struct.("wpts_slot_to_above_to_out_" + vString) = all_wpts;
end
