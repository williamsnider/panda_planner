clear; close all;
addpath("..")
params = CustomParameters();
[panda_ec, panda_sc] = loadPandaWithShape();
[env_norm, env_big] = build_collision_environment();

wd = pwd;
base_dir = wd+"/slots_struct/";
save_dir = wd+"/temp/";
dirItems = dir(base_dir);
fileNames = {dirItems(~[dirItems.isdir]).name};

% fileNames = {'20230915_10A28.mat'};

parfor i = 1:numel(fileNames)
    try
        fname = fileNames{i};
%         if ~strcmp(fname(1:11), "20230909_12")
%             continue
%         end

        savename = save_dir+fname(1:end-4);
        
        m = load(base_dir+fname);
        paths_struct = m.paths_struct;

        jointLimits = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits

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
        out_to_stagingA0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingA0, params);
        out_to_stagingB0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingB0, params);
        out_to_stagingC0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingC0, params);
        out_to_stagingD0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingD0, params);
        out_to_stagingE0 = joint_path_to_traj(paths_struct.wpts_out_to_stagingE0, params);

        paths_struct = []; % No longer needed

        % Trajectories - flipped
        out_to_home = flip(home_to_out,1);
        out_to_above = flip(above_to_out, 1);
        above_to_slot = flip(slot_to_above,1);
        stagingA0_to_out = flip(out_to_stagingA0,1);
        stagingB0_to_out = flip(out_to_stagingB0,1);
        stagingC0_to_out = flip(out_to_stagingC0,1);
        stagingD0_to_out = flip(out_to_stagingD0,1);
        stagingE0_to_out = flip(out_to_stagingE0,1);

        %% Add to traj struct
        traj_struct = setfield(traj_struct, "slot_to_above_"+sprintf("%02d",100*params.vScale), slot_to_above);
        traj_struct = setfield(traj_struct, "above_to_out_"+sprintf("%02d",100*params.vScale), above_to_out);
        traj_struct = setfield(traj_struct, "home_to_out_"+sprintf("%02d",100*params.vScale), home_to_out);
        traj_struct = setfield(traj_struct, "out_to_stagingA0_"+sprintf("%02d",100*params.vScale), out_to_stagingA0);
        traj_struct = setfield(traj_struct, "out_to_stagingB0_"+sprintf("%02d",100*params.vScale), out_to_stagingB0);
        traj_struct = setfield(traj_struct, "out_to_stagingC0_"+sprintf("%02d",100*params.vScale), out_to_stagingC0);
        traj_struct = setfield(traj_struct, "out_to_stagingD0_"+sprintf("%02d",100*params.vScale), out_to_stagingD0);
        traj_struct = setfield(traj_struct, "out_to_stagingE0_"+sprintf("%02d",100*params.vScale), out_to_stagingE0);
        traj_struct = setfield(traj_struct, "out_to_home_"+sprintf("%02d",100*params.vScale), out_to_home);
        traj_struct = setfield(traj_struct, "out_to_above_"+sprintf("%02d",100*params.vScale), out_to_above);
        traj_struct = setfield(traj_struct, "above_to_slot_"+sprintf("%02d",100*params.vScale), above_to_slot);
        traj_struct = setfield(traj_struct, "stagingA0_to_out_"+sprintf("%02d",100*params.vScale), stagingA0_to_out);
        traj_struct = setfield(traj_struct, "stagingB0_to_out_"+sprintf("%02d",100*params.vScale), stagingB0_to_out);
        traj_struct = setfield(traj_struct, "stagingC0_to_out_"+sprintf("%02d",100*params.vScale), stagingC0_to_out);
        traj_struct = setfield(traj_struct, "stagingD0_to_out_"+sprintf("%02d",100*params.vScale), stagingD0_to_out);
        traj_struct = setfield(traj_struct, "stagingE0_to_out_"+sprintf("%02d",100*params.vScale), stagingE0_to_out);


        assert(motionCheck(panda_ec, panda_sc,env_norm, traj_struct, params))
        writeToCSV(savename, params, traj_struct);

    catch
        disp("Failed for "+savename)
    end

end

