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

% Handle BottomShelf paths, which have additional upOut_to_aboveOut etc
if size(all_paths, 2) == 4
    paths_struct.("upOut_to_aboveOut_"+vString) = all_paths{4}(:,1:7);
    paths_struct.("aboveOut_to_upOut_"+vString) = flip(paths_struct.("upOut_to_aboveOut_" + vString), 1);
    paths_struct.("wpts_upOut_to_aboveOut_"+vString) = all_wpts(28:36,:);
    paths_struct.("wpts_downIn_to_upIn_to_upOut_to_aboveOut_"+vString) = [paths_struct.("wpts_downIn_to_downOut_" + vString); all_wpts(28:36,:)];

end


end
