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

quartet_fname = "/home/oconnorlabmatlab/Code/panda_planner/trajectory_planning/quartets/trajectories/20241009_A.mat";
save_dir = "/home/oconnorlabmatlab/Code/panda_planner/trajectory_planning/slots/trajectories/";
quartet_slots_csv = "/home/oconnorlabmatlab/Code/panda_planner/trajectory_planning/slots/quartet_slots.csv";
date_prefix = "20241009_";
slot_dir = "20241112_manual_slots";

quartet_slots = readcell(quartet_slots_csv);

for i = 1:numel(quartet_slots)
    slot_name = quartet_slots{i};

%     if strcmp(slot_name, "12C20") == false
%         continue
%     end



    shelf = slot_name(1:2);

    if ~strcmp(shelf,'12')
        continue
    end

    if (strcmp(shelf, '00')||strcmp(shelf,"01")||strcmp(shelf,"02")||strcmp(slot_name, "05A52")||strcmp(slot_name, "06A52")||strcmp(slot_name, "06C20")||strcmp(slot_name, "08C42"))
        disp("Skipping " + slot_name)
        continue
    end

    % Skip if already exists
    if checkSubstringInFilenames(save_dir, slot_name)
        disp("Skipping " + slot_name + " because it already exists.")
        continue
    end

    disp(slot_name)
    calcSlot(panda_ec, panda_sc, env, ik, slot_name, slot_dir, date_prefix, save_dir, quartet_fname, params);
end

