

function calcSlot(panda_ec, panda_sc, env, ik, slot_name, slot_dir, date_prefix, save_dir, quartet_fname, params)

% Load q given slot name
slot_fname = strcat(pwd,"/",slot_dir, "/",slot_name+".txt");
[q_slot, val] = readSlot(slot_fname);

% q_slot = [-0.648525,0.836017,0.202194,-1.47265,-2.87303,2.44462,0.424414];


% Create struct containing stagingA0a data
A = load(quartet_fname);
A = A.data_struct;
staging_data = struct();
idx = 1;  % Corresponds to staging id A0a;
staging_data.q_staging = A.staging_arr(idx,:);
staging_data.staging_id = A.names{idx};
staging_data.q_inter = A.inter_arr(idx,:);
staging_data.cell_staging_to_inter_path = A.cell_staging_to_inter_path{idx};

% Calculate trajectories relevant to each slot; store in paths_struct
stateBounds = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits
paths_struct = calcSlotTrajectories(panda_ec, panda_sc, env, ik, q_slot, stateBounds, staging_data, params);

% Write paths_struct fields to csv
traj_name_list = fieldnames(paths_struct);
for i = 1:numel(traj_name_list)
    traj_name = traj_name_list{i};
    
    % Skip wpts
    if contains(traj_name,"wpts")
        continue
    end

    % Write to csv
    traj = paths_struct.(traj_name);
    savename = strcat(save_dir, date_prefix, slot_name,"_",traj_name,"%.csv");
    
    writematrix(paths_struct.(traj_name), savename)
end
disp("Finished "+slot_name)

end

