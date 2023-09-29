% Add all subfolders in the libfranka/MATLAB directory to the path, to make
% it easier to use functions store in various folders

P = genpath(fileparts(mfilename('fullpath')));
P_split = strsplit(P, ":");

disp("Adding paths of libfranka/MATLAB folder ...")
for i=1:numel(P_split)
    curr_path = P_split{i};
    if contains(curr_path, 'archive') || isempty(curr_path)
        continue
    else
        disp(string(curr_path))
        addpath(string(curr_path))

    end
end

