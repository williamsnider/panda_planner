% Code that generates useful trajectories (everything not related to a
% specific slot)

clear; close all;
addpath("..")
params = CustomParameters();
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();
SAVE_DIR = "trajectories/20240423_";


%% Hold at home position

HOLD_TIME = 1000; % ms
hold_traj = repmat(params.q_home, [HOLD_TIME, 1]);
writematrix(hold_traj(:, 1:7), SAVE_DIR +'hold_home_'+HOLD_TIME+'ms.csv')

%% Paths between different staging positions

% Get list of all staging positions
stagingPositions = [];
fieldnamesList = fieldnames(params);
for f_num = 1:numel(fieldnamesList)
    f_name = fieldnamesList{f_num};
    if contains(f_name, "staging")
        stagingPositions = [stagingPositions; f_name];
    end
end

% % Generate pairwise paths (i.e. paths between all A's)
% stagingPositions = string(stagingPositions);  % Convert to string array
% for A_num = 1:numel(stagingPositions)
% 
%     for B_num = 1:numel(stagingPositions)
% 
%         A_name = stagingPositions(A_num,:);
%         B_name = stagingPositions(B_num,:);
% 
%         % Skip self
%         if A_name == B_name
%             continue
%         end
% 
%         % Skip different letters
%         A_name_char = char(A_name);
%         A_letter = A_name_char(end-1);
%         B_name_char = char(B_name);
%         B_letter = B_name_char(end-1);
% 
%         if A_letter ~= B_letter
%             continue
%         end
%         
%         path_name = A_name + "_to_" + B_name;
% 
%         % Skip if exists
%         if check_if_file_in_dir(path_name, "trajectories") == true
%             disp("Skipping " + path_name)
%             continue
%         end
% 
% 
% 
% %         disp("Increaing vMaxAll for joint7")
% %         orig_vMaxAll = params.vMaxAll(7);
% %         params.vMaxAll(7) = params.vMaxAllAbsolute(7)*3/4;  % increase vMaxAll
% 
%         % Plan paths
%         start = getfield(params, A_name);
%         goal = getfield(params, B_name);
%         all_trajectory = planJointToJoint(panda_ec, panda_sc, env, start, goal, params);
%         
%         % Check motion is OK
%         paths_struct = struct(path_name, all_trajectory);
%         assert(motionCheck(panda_ec, panda_sc, env, paths_struct, params))
% 
%         % Write CSV
%         writeCSV_from_paths_struct(paths_struct, SAVE_DIR, params);
% 
%         % return vMaxAll
% %         params.vMaxAll(7) = orig_vMaxAll;  
% 
% %         show(panda_ec, params.stagingA0)
% %         plotJointMotion(panda_sc, all_trajectory, env, params)
%     end
% end


%% Paths between home and staging
stagingPositions = string(stagingPositions);  % Convert to string array
parfor A_num = 1:numel(stagingPositions)


        A_name = stagingPositions(A_num,:);

        % Skip different letters
        A_name_char = char(A_name);
        A_letter = A_name_char(end-1);

        % Plan paths
        start = getfield(params, A_name);
        goal = getfield(params, "q_home");
        all_trajectory = planJointToJoint(panda_ec, panda_sc, env, start, goal, params);
        
        % Check motion is OK
        paths_struct = struct(A_name+"_to_home", all_trajectory, "home_to_"+A_name, flip(all_trajectory,1));
        assert(motionCheck(panda_ec, panda_sc, env, paths_struct, params))

        % Write CSV
        writeCSV_from_paths_struct(paths_struct, SAVE_DIR, params);

%         show(panda_ec, params.stagingA0)
%         plotJointMotion(panda_sc, all_trajectory, env, params)
end



