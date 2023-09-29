% Code to move the robot into the ballpark of these estimated positions
% (will need to be verified manually)

clear; close all;
addpath("..")
params = CustomParameters();

%% Inputs
SAVE_DIR = "missing/";

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();

%% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
weights = [1 1 1 1 1 1];
ss = manipulatorStateSpace(panda_ec); % for joint limits
stateBounds = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits


%% Generate paths to specified position

data_map = containers.Map('KeyType','int32','ValueType','any');

% Missing poses
load("missing_poses.mat", "missing_poses_map");
keys_list = missing_poses_map.keys;

count = 1;
for key_num = 1:numel(keys_list)

    % Read pose
    slot_name = keys_list{key_num};
    T_slot = missing_poses_map(slot_name);
    savename = SAVE_DIR+"missing"+slot_name;

    % Store pose
    vals = struct;
    vals.T_slot = T_slot;
    vals.savename = savename;
    vals.slot_name = slot_name;
    data_map(count) = vals;
    count = count + 1;
end

<<<<<<< HEAD
% Invalid slots as well
INVALID_DIR = "manual_slots_invalid/";
invalid_files = dir(INVALID_DIR);
for i = 1:numel(invalid_files)
    fname = invalid_files(i).name;

    if ~contains(fname, ".txt")
        continue
    end

    [~, val] = readSlot(INVALID_DIR+fname);

    T_slot = reshape(val.O_T_EE, [4,4]);
    slot_name = fname(1:5);
    savename = SAVE_DIR+"missing"+slot_name;

    % Store pose
    vals = struct;
    vals.T_slot = T_slot;
    vals.savename = savename;
    vals.slot_name = slot_name;
    data_map(count) = vals;
    count = count + 1;

end
=======
% % Invalid slots as well
% INVALID_DIR = "manual_slots_invalid/";
% invalid_files = dir(INVALID_DIR);
% for i = 1:numel(invalid_files)
%     fname = invalid_files(i).name;
% 
%     if ~contains(fname, ".txt")
%         continue
%     end
% 
%     [~, val] = readSlot(INVALID_DIR+fname);
% 
%     T_slot = reshape(val.O_T_EE, [4,4]);
%     slot_name = fname(1:5);
%     savename = SAVE_DIR+"missing"+slot_name;
% 
%     % Store pose
%     vals = struct;
%     vals.T_slot = T_slot;
%     vals.savename = savename;
%     vals.slot_name = slot_name;
%     data_map(count) = vals;
%     count = count + 1;
% 
% end

>>>>>>> f7d476bf33a95ad3d3a09c4998cfa0e71edc4515

parfor data_count = 1:data_map.Count
    vals = data_map(int32(data_count));
    T_slot = vals.T_slot;
    savename = vals.savename;

    %% Check if already calculated
    filenames = dir(SAVE_DIR);
    already_done = false;
    for i = 1:numel(filenames)
        fname = filenames(i).name;
        if contains(fname,vals.slot_name )
            already_done=true;
            break
        end
    end

    if already_done == true
        continue
    end

    % Loop multiple times since initial q_slot matters a lot
    valid_q = false;
    for q_attempt =1:1000
        try
            % Calculate q_slot
            initialGuess = randomConfiguration(panda_sc);
            weights = [1 1 1 1 1 1];
            [q_slot,solnInfo] = ik('panda_hand_tcp',T_slot,weights,initialGuess);

            % Test if q_slot not too close to joint limits
            scaled = (q_slot(1:7)-params.jointMin)./(params.jointMax-params.jointMin);
            thres = 0.02;
            if any(scaled>1.0-thres) || any(scaled<thres) || solnInfo.Status ~= "success"
                continue
            end


            [home_to_out, out_to_above, above_to_slot, slot_to_above, above_to_out, out_to_staging , staging_to_out, out_to_home] = calcSlotTrajectories(panda_ec, panda_sc, env, ik, q_slot, savename, stateBounds, params);
            combined = [home_to_out; out_to_above; above_to_slot; slot_to_above; above_to_out; out_to_staging ; staging_to_out; out_to_above; above_to_slot; slot_to_above; above_to_out; out_to_home];


            motions_ec = {home_to_out, out_to_staging, staging_to_out, out_to_home};
            assert(motionCheck(panda_ec, panda_sc,env, motions_ec, combined, params))
            valid_q = true;
            break
        catch
            valid_q = false;
            continue
        end
    end

    if valid_q == true
        writeToCSV(savename, params, home_to_out, out_to_above, above_to_slot, slot_to_above, above_to_out, out_to_staging, staging_to_out, out_to_home);
    else
        disp("Failed for "+savename)
    end

end






