% Code that generates useful trajectories (everything not related to a
% specific slot)

clear; close all;
addpath("../..")
params = CustomParameters();
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment();
prefix = '20241009_';
SAVE_DIR = 'trajectories/useful/';

mkdir(SAVE_DIR);
addpath(SAVE_DIR);

%% Generate list of staging positions 
A_struct = load(['trajectories/',prefix,'A.mat'],'data_struct');
A_struct = A_struct.data_struct;
% W_struct = load(['trajectories/',prefix,'W.mat'], 'data_struct');
% W_struct = W_struct.data_struct;


% %% Hold at home position
% 
% HOLD_TIME = 1000; % ms
% hold_traj = repmat(params.q_home, [HOLD_TIME, 1]);
% writematrix(hold_traj(:, 1:7), SAVE_DIR +'hold_home_'+HOLD_TIME+'ms.csv')


%% Paths between home and staging

num_positions = numel(A_struct.names);
for i = 1:1

    if i == 1
        data_struct = A_struct;
    else
        data_struct = W_struct;
    end

    for j = 1:num_positions

        name = data_struct.names{j};
        q_staging = data_struct.staging_arr(j,:);
        q_inter = data_struct.inter_arr(j,:);
        q_goal = params.q_home;

        % Sanity check
        assert(sum(abs(q_staging-q_inter(1,:)))<1.0);

        % Plan path from q_staging_to q_inter to q_home  at 70% max speed      
        params_copy = params;
        params_copy.vScale = 0.7;
        params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
        [~, planned_path_inter_to_home] = planJointToJoint(panda_ec, panda_sc, env, q_inter, q_goal, params_copy);
        
        % Check that inter is the same and combine
        planned_path_staging_to_inter = data_struct.cell_staging_to_inter_path{j};
        assert(sum(sum(abs(planned_path_staging_to_inter(end,:)-planned_path_inter_to_home(1,:))))<0.001)
    

        planned_path_staging_inter_home = [planned_path_staging_to_inter(1:end-1,:);planned_path_inter_to_home]; % Add q_staging to inter positionn
        
        [traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(planned_path_staging_inter_home, panda_sc,params);
        
        % Write to CSV
        fname_base = strcat(SAVE_DIR,prefix, "staging", name,"_to_home")
        write_to_CSV_10_40_70(traj_10,traj_40,traj_70,fname_base)
        fname_base = strcat(SAVE_DIR, prefix,"home_to_staging",name);
        write_to_CSV_10_40_70(traj_10_reverse,traj_40_reverse,traj_70_reverse,fname_base)



    end
end





% parfor A_num = 1:numel(stagingPositions)
% 
% 
%         A_name = stagingPositions(A_num,:);
% 
%         % Skip different letters
%         A_name_char = char(A_name);
%         A_letter = A_name_char(end-1);
% 
%         % Plan paths
%         start = getfield(params, A_name);
%         goal = getfield(params, "q_home");
%         all_trajectory = planJointToJoint(panda_ec, panda_sc, env, start, goal, params);
%         
%         % Check motion is OK
%         paths_struct = struct(A_name+"_to_home", all_trajectory, "home_to_"+A_name, flip(all_trajectory,1));
%         assert(motionCheck(panda_ec, panda_sc, env, paths_struct, params))
% 
%         % Write CSV
%         writeCSV_from_paths_struct(paths_struct, SAVE_DIR, params);
% 
% %         show(panda_ec, params.stagingA0)
% %         plotJointMotion(panda_sc, all_trajectory, env, params)
% end



