% Code to move the robot into the ballpark of these estimated positions
% (will need to be verified manually)

clear; close all;
addpath("../..")
params = CustomParameters();

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
[env_norm, env_big] = build_collision_environment();
% ss = ManipulatorStateSpaceSphere(panda_ec, panda_sc);
% sv = ManipulatorStateValidatorSphere(ss, env, params.validationDistance, 0, params);
%

% Get filenames
slots_struct_dir = "./20230907/";
date_prefix = "20230907";
all_files = dir(slots_struct_dir);
mat_files = {all_files(~[all_files.isdir] & endsWith({all_files.name}, '.mat')).name};

% Get all filenames starting with 06B
sub_files = mat_files(startsWith(mat_files, '20230907_06B'));

% Get combinations of 2
comb_indices = nchoosek(1:length(sub_files), 2);
comb_files = cell(size(comb_indices, 1), 1);
for i = 1:size(comb_indices, 1)
    comb_files{i} = {sub_files{comb_indices(i, 1)}, sub_files{comb_indices(i, 2)}};
end

% Plan motion between them
parfor i = 1:numel(comb_files)
    comb = comb_files{i};
    A = comb{1};
    B = comb{2};

    %     if ~strcmp(A, "06B34.mat") && ~strcmp(B, "06B45.mat")
    %         continue
    %     end

    try


        mA = load(strcat(slots_struct_dir, A));
        mB = load(strcat(slots_struct_dir, B));

        pA = mA.paths_struct;
        pB = mB.paths_struct;

        % Get q of out position
        qA = pA.out_to_home(1,:);
        qB = pB.out_to_home(1,:);

        % Labels and filename
        trajA_to_B = strcat("slot",A(10:14),"_to_slot",B(10:14),"_",sprintf("%02d", params.vScale*100));
        trajB_to_A = strcat("slot",B(10:14),"_to_slot", A(10:14),"_",sprintf("%02d", params.vScale*100));
        wpts_A_to_B = strcat("wpts_slot",A(10:14),"_to_slot",B(10:14));
        struct_name = char(trajA_to_B);
        struct_name = struct_name(1:end-3);
        fname = strcat("./slots_struct/"+date_prefix+"_" , struct_name, ".mat");

        % Test if file already exists. If so, no need to recalculate path
        already_exists = exist(fname,"file")==2;
        if already_exists
            m = load(fname);
            path_struct = m.path_struct;
            planned_path = getfield(path_struct, wpts_A_to_B);
        else
            planned_path = joint_plan_path(panda_ec, panda_sc, env_big, qA, qB, params);
            path_struct=struct(wpts_A_to_B, planned_path);
        end


        % Plan path between A and B

        % Calculate trajectory for planned path at desired velocity (set in
        % CustomParameters)
        traj = joint_path_to_traj(planned_path, params);

        traj_struct = struct;
        traj_struct = setfield(traj_struct, trajA_to_B, traj);
        traj_struct = setfield(traj_struct, trajB_to_A, flip(traj,1));

        % Check the motion - CRITICAL.
        assert(motionCheck(panda_ec, panda_sc,env_norm, traj_struct, params))


        % Save paths_struct (can reuse for different speeds)
        if ~already_exists

            m=matfile(fname,'writable',true);
            m.path_struct = path_struct;
        end

        writeToCSV("./slots/"+date_prefix, params, traj_struct);

    catch
        disp(strcat("Failure for ",A," and ",B))
    end

end
