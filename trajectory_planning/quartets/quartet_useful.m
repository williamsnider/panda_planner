% Code that generates useful trajectories (everything not related to a
% specific slot)

clear; close all;
addpath("..")
params = CustomParameters();
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment();
prefix = '20240728_';
SAVE_DIR = ['paths/useful/',prefix];

mkdir(SAVE_DIR);
addpath(SAVE_DIR);

%% Generate list of staging positions 
A= load(['paths/',prefix,'A.mat']);
qA = qA.qA;
qW = load(['paths/',prefix,'W.mat'], "qW");
qW = qW.qW;

% Names
qA_letters = ["A","B","C","D"];
qA_names =  generate_staging_names(qA_letters);

qW_letters = ["W","X","Y","Z"];
qW_names =  generate_staging_names(qW_letters);


% %% Hold at home position
% 
% HOLD_TIME = 1000; % ms
% hold_traj = repmat(params.q_home, [HOLD_TIME, 1]);
% writematrix(hold_traj(:, 1:7), SAVE_DIR +'hold_home_'+HOLD_TIME+'ms.csv')


%% Paths between home and staging

num_positions = size(qA.qA_arr,1);
for i = 1:2

    if i == 1
        names = qA_names;
        arr = qA.qA_arr;
        inter = qA.cell_staging_to_inter_path;
    else
        names = qW_names;
        arr = qW.qW_arr;
        inter = qW.cell_staging_to_inter_path;
    end

    parfor j = 1:num_positions

        name = names{j};
        q_staging = arr(j,:);
        q_inter = inter{j}(2,:);
        goal = params.q_home;

        % Sanity check
        assert(sum(abs(q_staging-inter{j}(1,:)))<0.001);

        % Plan path from q_staging_to q_inter to q_home  at 70% max speed      
        params_copy = params;
        params_copy.vScale = 0.7;
        params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
        [~, planned_path] = planJointToJoint(panda_ec, panda_sc, env, q_inter, goal, params_copy);
        planned_path = [q_staging;planned_path]; % Add q_staging to inter positionn
        
        [traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(planned_path, panda_sc,params)
        
        % Write to CSV
        fname_base = strcat(SAVE_DIR, "staging", name,"_to_home");
        write_to_CSV_10_40_70(traj_10,traj_40,traj_70,fname_base)
        fname_base = strcat(SAVE_DIR, "home_to_staging",name)
        write_to_CSV_10_40_70(traj_10_reverse,traj_40_reverse,traj_70_reverse,fname_base)



    end
end


%% Paths between W and A

for A_idx = 1:2
    A_staging = qA.qA_arr(A_idx,:);
    A_inter = qA.cell_staging_to_inter_path{A_idx}(2,:);
    A_name = qA_names{A_idx};

    for W_idx = 1:2
        W_staging = qW.qW_arr(W_idx,:);
        W_inter = qW.cell_staging_to_inter_path{W_idx}(2,:);
        W_name = qW_names{W_idx};
        
        % Calculate path between inter
        params_copy = params;
        params_copy.vScale = 0.7;
        params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
        [~, planned_path] = planJointToJoint(panda_ec, panda_sc, env, A_inter, W_inter, params_copy);
        planned_path = [A_staging;planned_path;W_staging];
        
        % CalculaTe trajectories at 10, 40, 70% max speed
        [traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(planned_path, panda_sc,params);
        
        % Write to CSV - forward
        fname_base = strcat(SAVE_DIR, "staging", A_name,"_to_","staging",W_name);
        write_to_CSV_10_40_70(traj_10, traj_40, traj_70, fname_base);
        
        % Write to CSV - reverse
        fname_base = strcat(SAVE_DIR, "staging", W_name,"_to_","staging",A_name);
        write_to_CSV_10_40_70(traj_10_reverse, traj_40_reverse, traj_70_reverse, fname_base);
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



