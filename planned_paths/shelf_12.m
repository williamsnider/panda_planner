% Code to move the robot into the ballpark of these estimated positions
% (will need to be verified manually)

clear; close all;
addpath("../..")
params = CustomParameters();

%% Inputs
SAVE_DIR = "missing/";

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
[env_norm, env_big] = build_collision_environment();

%% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
weights = [1 1 1 1 1 1];
ss = manipulatorStateSpace(panda_ec); % for joint limits
stateBounds = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits
jointLimits = stateBounds;

%% List of Shelf 12 poses
directory_path = "/home/oconnorlab/Code/libfranka/MATLAB/planned_paths/manual_slots";
file_list = dir(fullfile(directory_path, '08*.txt'));

pose_map = containers.Map("keyType", 'int64', 'ValueType', 'any');

for i = 1:numel(file_list)
    fname = file_list(i);
    q = readSlot(fname.folder+"/"+fname.name);
    T_slot_original = getTransform(panda_ec, q, "panda_hand_tcp");
    T_slot = T_slot_original;
%     T_slot(3,4) = T_slot(3,4) + 0.015; % Increase height 15mm

% Rotate by 5 degrees about second column
    R = T_slot(1:3,1:3);
    v = R(:, 2); % extract the second column vector
    theta = -5; % rotation angle in degrees
    R_rotate = rodrigues_rotation(v, theta);
    R_new  =R_rotate*R;
    T_slot(1:3,1:3) = R_new;

    slot_name = fname.name(1:5);

    vals = struct;
    vals.T_slot = T_slot;
    vals.savename = SAVE_DIR+"missing"+slot_name;
    vals.slot_name = slot_name;
    pose_map(i) = vals;
end

parfor data_count =1:pose_map.Count
    vals = pose_map(int32(data_count));
    T_slot = vals.T_slot;
    savename = vals.savename;

        %% Check if already calculated
        filenames = dir(SAVE_DIR);
        already_done = false;
        for i = 1:numel(filenames)
            fname = filenames(i);
            fname = fname.name;
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
%         disp(q_attempt)
        try
            % Calculate q_slot
            initialGuess = randomConfiguration(panda_sc);
            weights = [1 1 1 1 1 1];
            [q_slot,solnInfo] = ik('panda_hand_tcp',T_slot,weights,initialGuess);

            % Test if q_slot not too close to joint limits
            scaled = (q_slot(1:7)-params.jointMin)./(params.jointMax-params.jointMin);
            thres = 0.05;
            if any(scaled>1.0-thres) || any(scaled<thres) || solnInfo.Status ~= "success"
                continue
            end


            paths_struct= calcSlotTrajectories(panda_ec, panda_sc, env_big, ik, q_slot, savename, stateBounds, params);

            wpts_slot_to_above = paths_struct.wpts_slot_to_above;
            wpts_above_to_out = paths_struct.wpts_above_to_out;


            traj_struct = struct;
            %% Cartesian trajectories
            aFactor = 0.9;
            [slot_to_above, valid] = cartesian_wpts_to_traj(wpts_slot_to_above, aFactor, panda_ec, panda_sc, env_norm, jointLimits, params);
            assert(valid)
            [above_to_out, valid] = cartesian_wpts_to_traj(wpts_above_to_out, aFactor, panda_ec, panda_sc, env_norm, jointLimits, params);
            assert(valid)

            %% Joint trajectories
            home_to_out = joint_path_to_traj(paths_struct.wpts_home_to_out, params);
%             out_to_stagingA0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingA0, params);
%             out_to_stagingB0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingB0, params);
%             out_to_stagingC0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingC0, params);
%             out_to_stagingD0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingD0, params);
%             out_to_stagingE0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingE0, params);

            paths_struct = []; % No longer needed

            % Trajectories - flipped
            out_to_home = flip(home_to_out,1);
            out_to_above = flip(above_to_out, 1);
            above_to_slot = flip(slot_to_above,1);
%             stagingA0_to_out = flip(out_to_stagingA0,1);
%             stagingB0_to_out = flip(out_to_stagingB0,1);
%             stagingC0_to_out = flip(out_to_stagingC0,1);
%             stagingD0_to_out = flip(out_to_stagingD0,1);
%             stagingE0_to_out = flip(out_to_stagingE0,1);

            %% Add to traj struct
            traj_struct = setfield(traj_struct, "slot_to_above_"+sprintf("%02d",100*params.vScale), slot_to_above);
            traj_struct = setfield(traj_struct, "above_to_out_"+sprintf("%02d",100*params.vScale), above_to_out);
            traj_struct = setfield(traj_struct, "home_to_out_"+sprintf("%02d",100*params.vScale), home_to_out);
%             traj_struct = setfield(traj_struct, "out_to_stagingA0_"+sprintf("%02d",100*params.vScale), out_to_stagingA0);
%             traj_struct = setfield(traj_struct, "out_to_stagingB0_"+sprintf("%02d",100*params.vScale), out_to_stagingB0);
%             traj_struct = setfield(traj_struct, "out_to_stagingC0_"+sprintf("%02d",100*params.vScale), out_to_stagingC0);
%             traj_struct = setfield(traj_struct, "out_to_stagingD0_"+sprintf("%02d",100*params.vScale), out_to_stagingD0);
%             traj_struct = setfield(traj_struct, "out_to_stagingE0_"+sprintf("%02d",100*params.vScale), out_to_stagingE0);
%             traj_struct = setfield(traj_struct, "out_to_home_"+sprintf("%02d",100*params.vScale), out_to_home);
            traj_struct = setfield(traj_struct, "out_to_above_"+sprintf("%02d",100*params.vScale), out_to_above);
            traj_struct = setfield(traj_struct, "above_to_slot_"+sprintf("%02d",100*params.vScale), above_to_slot);
%             traj_struct = setfield(traj_struct, "stagingA0_to_out_"+sprintf("%02d",100*params.vScale), stagingA0_to_out);
%             traj_struct = setfield(traj_struct, "stagingB0_to_out_"+sprintf("%02d",100*params.vScale), stagingB0_to_out);
%             traj_struct = setfield(traj_struct, "stagingC0_to_out_"+sprintf("%02d",100*params.vScale), stagingC0_to_out);
%             traj_struct = setfield(traj_struct, "stagingD0_to_out_"+sprintf("%02d",100*params.vScale), stagingD0_to_out);
%             traj_struct = setfield(traj_struct, "stagingE0_to_out_"+sprintf("%02d",100*params.vScale), stagingE0_to_out);


            assert(motionCheck(panda_ec, panda_sc,env_norm, traj_struct, params))
            valid_q = true;
            break
        catch
            valid_q = false;
            continue
        end
    end

    if valid_q == true
        writeToCSV(savename, params, traj_struct)
    else
        disp("Failed for "+savename)
    end

end






