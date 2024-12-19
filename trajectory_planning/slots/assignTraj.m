function paths_struct = assignTraj(paths_struct, all_paths, all_wpts, vScale)

% Convert vScale to string
vString = num2str(round(100*vScale));

paths_struct.("downIn_to_upIn_" + vString) = all_paths{1}(:,1:7);
paths_struct.("upIn_to_downIn_" + vString) = flip(paths_struct.("downIn_to_upIn_" + vString), 1);
paths_struct.("wpts_downIn_to_upIn_" + vString) = all_wpts(1:9, :);

paths_struct.("upIn_to_upOut_" + vString) = all_paths{2}(:,1:7);
paths_struct.("upOut_to_upIn_" + vString) = flip(paths_struct.("upIn_to_upOut_" + vString), 1);
paths_struct.("wpts_upIn_to_upOut_" + vString) = all_wpts(10:18, :);

paths_struct.("downIn_to_downOut_" + vString) = all_paths{3}(:,1:7);
paths_struct.("downOut_to_downIn_" + vString) = flip(paths_struct.("downIn_to_downOut_"+vString), 1);
paths_struct.("wpts_downIn_to_downOut_" + vString) = all_wpts(19:27,:);

paths_struct.("wpts_downIn_to_upIn_to_upOut_" + vString) = all_wpts(1:18,:);
end
