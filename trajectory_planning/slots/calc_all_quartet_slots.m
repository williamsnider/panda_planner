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
date_prefix = "20241009_";
slot_dir = "20241112_manual_slots";

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

quartet_slots = readcell(quartet_slots_csv);
valid_slots = {};

% Pre-filter slots before parallel execution
for i = 1:numel(quartet_slots)
    slot_name = quartet_slots{i};
    shelf = slot_name(1:2);

    if ~strcmp(shelf, '00')
        continue
    end

%     if (strcmp(shelf, '00') || strcmp(shelf, "01") || strcmp(shelf, "02") || ...
%         strcmp(slot_name, "05A52") || strcmp(slot_name, "06A52") || ...
%         strcmp(slot_name, "06C20") || strcmp(slot_name, "08C42"))
%         disp("Skipping " + slot_name)
%         continue
%     end

    % Skip if already exists
    if checkSubstringInFilenames(save_dir, slot_name)
        disp("Skipping " + slot_name + " because it already exists.")
        continue
    end

    valid_slots{end+1} = slot_name;
end

% valid_slots{end+1} = "06C08";

% Parallel execution
for i = 1:numel(valid_slots)
    slot_name = valid_slots{i};
    disp(slot_name)
    calcSlot(panda_ec, panda_sc, env, ik, slot_name, slot_dir, date_prefix, save_dir, quartet_fname, params);
end



