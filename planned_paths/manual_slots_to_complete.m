% Code to move the robot to slots on shelf panel 01B
clear; close all;
addpath("..")
params = CustomParameters();

%% Inputs
dir_cell = split(pwd, "/");
assert(strcmp(dir_cell{end},'planned_paths'), "Directory must be MATLAB/planned_paths")
SAVE_DIR = "slots/";
SLOTS_DIR = "manual_slots/";
date_prefix = "20230926_";

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
[env_norm, env_big] = build_collision_environment();

%% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
weights = [1 1 1 1 1 1];
stateBounds = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits

%% Generate paths to specified position
data_map = containers.Map('KeyType','uint64','ValueType','any');
filenames = dir(SLOTS_DIR);
count = 1;
for idx = 1:numel(filenames)

    fname = filenames(idx);

    % Skip . and ..
    if contains(fname.name, ".txt") == false
        continue
    end

    % Read q
    q_slot = readSlot(fname.folder + "/" + fname.name);

    % Generate name
    slot_name = fname.name(1:end-4);  % remove .txt
    savename = SAVE_DIR+date_prefix+slot_name;


    % Calc coordinates of shape on shelf
    vals = struct;
    vals.q = q_slot;
    vals.savename = savename;
    vals.slot_name = slot_name;
    data_map(count) = vals;
    count = count+1;


end

num_loops = data_map.Count;
parfor loop_idx = 1:num_loops
%     data_count = num_loops - loop_idx+1;
    data_count = loop_idx;
    vals = data_map(uint64(data_count));
    q_slot = vals.q;
    savename = vals.savename;
    slot_name = vals.slot_name;

%         if ~strcmp(slot_name, "12A00")
%             continue
%         end
    slot_name

    %% Check if already calculated
    filenames = dir("slots");
    already_done = false;
    for i = 1:numel(filenames)
        fname = filenames(i).name;
        if contains(fname,slot_name)
            already_done=true;
            break
        end
    end

    if already_done
        disp("Skipping " +slot_name)
        continue
    end

    %% Calculate trajectories

    % try/catch block to help parfor loop. Rerunning this often solves the
    % issue (likely an initial ik solution not optimal).
    try
        paths_struct = calcSlotTrajectories(panda_ec, panda_sc, env_norm, ik, q_slot, savename, stateBounds, params);

        % Check the motion - CRITICAL.
        assert(motionCheck(panda_ec, panda_sc,env_norm, paths_struct, params))

        % Save paths_struct (can reuse for different speeds)
        fname = strcat("./slots_struct/" ,date_prefix, slot_name, ".mat");
        m=matfile(fname,'writable',true);
        m.paths_struct = paths_struct;

        writeToCSV(savename, params, paths_struct);

        % Move to valid folder
        movefile("manual_slots/" + slot_name+".txt", "manual_slots_valid/"+slot_name+".txt")
    catch
        disp(vals.slot_name+" FAILED TO FIND VALID TRAJECTORIES")
        movefile("manual_slots/" + slot_name+".txt", "manual_slots_invalid/"+slot_name+".txt")
    end
end


%
% filename = "slots/01B00_out_to_above_5%.csv";
% show(panda_sc, params.q_home)
% plotCSV(panda_sc, filename, env, params)
