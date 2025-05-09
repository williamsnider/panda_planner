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
date_prefix = "20250506_";
slot_dir = "20250508_manual_slots";

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

quartet_slots = readcell(quartet_slots_csv);
valid_slots = {};

% % Pre-filter slots before parallel execution
% for i = 1:numel(quartet_slots)
%     slot_name = quartet_slots{i};
%     shelf = slot_name(1:2);
% 
% 
% 
% 
% %     if strcmp(slot_name, "06C20") || ... % Joint limit reached
% %             strcmp(slot_name, "04C32") || ... % Joint limit reached
% %             strcmp(slot_name, "08C42") || ... % Cartesian calculation fails
% %             strcmp(slot_name, "09A30") || ... % Cartesian calculation fails
% %                     strcmp(slot_name, "05C32")  % Cartesian calculation fails
% % 
% %         disp("Skipping " + slot_name)
% %         continue
% %     end
% 
%     % Skip if already exists
%     if checkSubstringInFilenames(save_dir, slot_name)
%         disp("Skipping " + slot_name + " because it already exists.")
%         continue
%     end
% 
%     valid_slots{end+1} = slot_name;
% end
% 
valid_slots{end+1} = '04C32';
% % valid_slots{end+1} = '05C32';
% % valid_slots{end+1} = '06A52';
valid_slots{end+1} = '06C20';
% % valid_slots{end+1} = '07C46';
% % valid_slots{end+1} = '08A18';
% % valid_slots{end+1} = '08C42';
% % valid_slots{end+1} = '09A18';
% % valid_slots{end+1} = '09A30';

% % valid_slots{end+1} = '00C28';
% % valid_slots{end+1} = '01A02';
% % valid_slots{end+1} = '01A16';
% % valid_slots{end+1} = '01B40';
% % valid_slots{end+1} = '02B04';
% % valid_slots{end+1} = '03A18';
% % valid_slots{end+1} = '03A30';
% % valid_slots{end+1} = '03A42';
% % valid_slots{end+1} = '03B02';
% % valid_slots{end+1} = '07B36';
% % valid_slots{end+1} = '08A06';
% % valid_slots{end+1} = '08A42';
% % valid_slots{end+1} = '09A42';
% % valid_slots{end+1} = '09B06';
% % valid_slots{end+1} = '09B30';
% % valid_slots{end+1} = '09C30';
% % valid_slots{end+1} = '10B28';
% % valid_slots{end+1} = '10C44';


% Parallel execution
for i = 1:numel(valid_slots)
    slot_name = valid_slots{i};
    disp(slot_name)
    
    calcSlot(panda_ec, panda_sc, env, ik, slot_name, slot_dir, date_prefix, save_dir, quartet_fname, params);
end



