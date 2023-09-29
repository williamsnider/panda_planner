% % Read the joint position from the text file
% 
% clear; close all;
% addpath("..")
% params = CustomParameters();
% 
% fname = "/home/oconnorlabmatlab/Code/libfranka/build/slots/01C02.txt";
% savename = "/home/oconnorlabmatlab/Code/libfranka/build/slots/test_out";
% 
% 
% 
% %% Inputs
% SAVE_DIR = "slots/";
% 
% %% Load robot
% [panda_ec, panda_sc] = loadPandaWithShape();
% env = build_collision_environment();
% 
% %% Set up inverse kinematics
% ik = inverseKinematics('RigidBodyTree',panda_sc);
% ik.SolverParameters.MaxIterations = 1000;
% weights = [1 1 1 1 1 1];
% ss = manipulatorStateSpace(panda_ec); % for joint limits
% stateBounds = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits
% 
% 
% 
% % Calculate path
% [home_to_out, out_to_above, above_to_slot, slot_to_above, above_to_out, out_to_staging , staging_to_out, out_to_home] = calcSlotTrajectories(panda_ec, panda_sc, env, ik, q_slot, savename, stateBounds, params);
% combined = [home_to_out; out_to_above; above_to_slot; slot_to_above; above_to_out; out_to_staging ; staging_to_out; out_to_above; above_to_slot; slot_to_above; above_to_out; out_to_home];
% 
% 
% motions_ec = {home_to_out, out_to_staging, staging_to_out, out_to_home};
% assert(motionCheck(panda_ec, panda_sc,env, motions_ec, combined, params))
% valid_q = true;

function [q_slot, val] = readSlot(fname)

% Extract json
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);

% Extract q
q_slot = val.q';
q_slot = [q_slot, 0.01, 0.01];  % Add finger positions
end