%% Clear all variables and loam parameters
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Load variables
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment;
ik = inverseKinematics('RigidBodyTree', panda_sc);
ik.SolverParameters.MaxIterations = 1000;

quartet_fname = params.CustomParametersDir+"/trajectory_planning/quartets/trajectories/20241009_A.mat";
save_dir = params.CustomParametersDir+"/trajectory_planning/slots/trajectories/";
quartet_slots_csv = params.CustomParametersDir+"/trajectory_planning/slots/quartet_slots.csv";
date_prefix = "20250506_";
slot_dir = "20250509_manual_slots";

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

quartet_slots = readcell(quartet_slots_csv);
valid_slots = {};




files = dir(fullfile(save_dir, date_prefix+'*upOut_to_stagingA0a_10%.csv'));
slot_data = struct('slot_name', {}, 'q_upOut', {});

for k = 1:length(files)
    filepath = fullfile(save_dir, files(k).name);
    filepath_split = split(files(k).name, "_");  % Use name, not full path
    slot_name = filepath_split(end-4);

    % Skip 00 shelf because it has intermediate position (need to
    % implement)
    if strcmp(slot_name{1}(1:2), "00")
        disp("Skipping " + slot_name)
        continue
    end

    data = readmatrix(filepath);
    if ~isempty(data)
        slot_data(end+1).slot_name = slot_name;
        slot_data(end).q_upOut = [data(1, :), 0.01, 0.01];
    end
end


% Calculate pairs
for a=1:numel(slot_data)-1
    
    slot_name_A = slot_data(a).slot_name;
    q_upOut_A = slot_data(a).q_upOut;

    for b=a+1:numel(slot_data)

        % Calculate path
        slot_name_B = slot_data(b).slot_name;
        q_upOut_B = slot_data(b).q_upOut;

        paths_struct.("wpts_"+slot_name_A+"_upOut_to_"+slot_name_B+"_upOut") = joint_plan_path(panda_ec, panda_sc, env, q_upOut_A, q_upOut_B, params);
        [traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.("wpts_"+slot_name_A+"_upOut_to_"+slot_name_B+"_upOut") , panda_sc,params);
        paths_struct.("pair_"+slot_name_A+"_upOut_to_"+slot_name_B+"_upOut_10") = traj_10;
        paths_struct.("pair_"+slot_name_A+"_upOut_to_"+slot_name_B+"_upOut_40") = traj_40;
        paths_struct.("pair_"+slot_name_A+"_upOut_to_"+slot_name_B+"_upOut_70") = traj_70;
        paths_struct.("pair_"+slot_name_B+"_upOut_to_"+slot_name_A+"_upOut_10") = traj_10_reverse;
        paths_struct.("pair_"+slot_name_B+"_upOut_to_"+slot_name_A+"_upOut_40") = traj_40_reverse;
        paths_struct.("pair_"+slot_name_B+"_upOut_to_"+slot_name_A+"_upOut_70") = traj_70_reverse;

        % Write paths struct to csv
        traj_name_list = fieldnames(paths_struct);
        for i = 1:numel(traj_name_list)
            traj_name = traj_name_list{i};

            % Skip wpts
            if contains(traj_name,"wpts")
                continue
            end

            % Write to csv
            traj = paths_struct.(traj_name);
            traj_name_sans_pair = erase(traj_name, "pair_");
            savename = strcat(save_dir, date_prefix,traj_name_sans_pair,"%.csv");

            writematrix(paths_struct.(traj_name), savename)
        end
        disp("Finished "+traj_name_sans_pair)
    

    end

end

