% Code to move the robot into the ballpark of these estimated positions
% (will need to be verified manually)

clear; close all;
addpath("..")
params = CustomParameters();

%% Inputs
SAVE_DIR = "ballpark/";

Z_DOWNSHIFT = -0.02
disp("Shifting down Z slightly so that shape does not collide.")
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

th_list = linspace(-3*pi/4, 3*pi/4, 21);

count =1;
for shelf_num = 0:numel(params.Z_list)-1
    for th_num = 1:numel(th_list)
        th = th_list(th_num);


        % Generate name
        %         th_name = num2str(th, 2);
        %         th_name = strrep(th_name, '.', 'p');
        th_name = num2str(th_num);
        bp_name = "_shelf_"+num2str(shelf_num)+"_and_th_"+th_name;
        savename = SAVE_DIR+"ballpark"+bp_name;

        % Calc coordinates of shape on shelf
        z = params.Z_list(shelf_num+1)+Z_DOWNSHIFT;
        radius = params.radius_list(shelf_num+1);
        x = radius * cos(th);
        y = radius * sin(th);

        T_base = eul2tform([0, pi/2,-th+params.SHAPE_ROTATION_ABOUT_Z]); % 180 deg rotation of end effector
        T_slot = T_base;
        T_slot(1:3,4) = [x,y,z];

        vals = struct;
        vals.T_slot = T_slot;
        vals.savename = savename;
        vals.bp_name = bp_name;
        data_map(count) = vals;
        count = count + 1;

    end
end

parfor data_count = 1:data_map.Count
    vals = data_map(int32(data_count));
    T_slot = vals.T_slot;
    savename = vals.savename;
    bp_name = vals.bp_name;

    %% Check if already calculated
    filenames = dir("ballpark");
    already_done = false;
    for i = 1:numel(filenames)
        fname = filenames(i).name;
        if contains(fname,vals.bp_name )
            already_done=true;
            break
        end
    end

    if already_done == true
        continue
    end

    % Loop multiple times since initial q_slot matters a lot
    valid_q = false;
    for q_attempt =1:200
        try
            % Calculate q_slot
            initialGuess = randomConfiguration(panda_sc);
            weights = [1 1 1 1 1 1];
            [q_slot,solnInfo] = ik('panda_hand_tcp',T_slot,weights,initialGuess);

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


filename = "missing/missing00A00_home_to_out_10%.csv";
show(panda_sc, params.q_home)
plotCSV(panda_sc, filename, env, params)

% show(panda_sc, params.q_home)
% plotJointMotion(panda_sc, combined, env, params.sphere_radius, params.sphere_origin)
% plotJointMotion(panda_sc, above_to_out, env, sphere_radius, sphere_origin)
% Procedure for teaching robot slot positions
% - Move robot to ballpark of slot (T_above_and_out and T_above)
% - Manually guide robot with shape into slot, record this position
% - Use recorded position to calculate an exact path


