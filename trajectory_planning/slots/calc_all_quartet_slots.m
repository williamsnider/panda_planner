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
date_prefix = "20250618_";
slot_dir = "20250618_manual_slots";

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

quartet_slots = readcell(quartet_slots_csv);
valid_slots = {};

% Pre-filter slots before parallel execution
for i = 1:numel(quartet_slots)
    slot_name = quartet_slots{i};
    shelf = slot_name(1:2);

    % Skip if already exists
    if checkSubstringInFilenames(save_dir, slot_name)
        disp("Skipping " + slot_name + " because it already exists.")
        continue
    end

    valid_slots{end+1} = slot_name;
end

disp(strcat("Number of slots left: ", num2str(numel(valid_slots))))


% valid_slots{end+1} = '07C32';
% % valid_slots{end+1} = '11A04';
p = gcp('nocreate'); 
if ~isempty(p)
    delete(p);
end
parpool('local', 12);


% Parallel execution
for i = 1:numel(valid_slots)
    slot_name = valid_slots{i};
    disp(slot_name)
    try
        calcSlot(panda_ec, panda_sc, env, ik, slot_name, slot_dir, date_prefix, save_dir, quartet_fname, params);
    catch
        disp(strcat("Failed for ", slot_name))
    end
end



