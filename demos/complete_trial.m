clear; close all;
run /home/williamsnider/Code/libfranka/MATLAB/add_all_paths.m
run /home/williamsnider/Code/libfranka/MATLAB/parameters.m

panda = loadPandaWithShape();
env = build_collision_environment();


% Load data
filename = ['/home/williamsnider/Code/libfranka/MATLAB/kinematics/trajectory/data/slot_', num2str(slotNum),'.mat'];
load(filename)

% Parse present into different stages
start = 1;
stop = start-1+startToMonkeyTime*controlHz;
anglesStagingToMonkey = anglesPresent(start:stop, :);
start = stop+1;
stop = start-1+monkeyHoldTime*controlHz;
anglesHold = anglesPresent(start:stop, :);
start = stop+1;
stop = start-1+monkeyToPullTime*controlHz;
anglesMonkeyToPull = anglesPresent(start:stop, :);
start = stop+1;
stop = start-1+pullToStartTime*controlHz;
anglesPullToStaging = anglesPresent(start:stop, :);

% Load real data
home_to_out = readmatrix('/home/williamsnider/Code/libfranka/MATLAB/planned_paths/Slot5_home_to_out_70%.csv');
out_to_above = readmatrix('/home/williamsnider/Code/libfranka/MATLAB/planned_paths/Slot5_out_to_above_70%.csv');
above_to_slot = readmatrix('/home/williamsnider/Code/libfranka/MATLAB/planned_paths/Slot5_above_to_slot_70%.csv');
slot_to_above = readmatrix('/home/williamsnider/Code/libfranka/MATLAB/planned_paths/Slot5_slot_to_above_70%.csv');
above_to_out = readmatrix('/home/williamsnider/Code/libfranka/MATLAB/planned_paths/Slot5_above_to_out_70%.csv');
out_to_staging = readmatrix('/home/williamsnider/Code/libfranka/MATLAB/planned_paths/Slot5_out_to_staging_70%.csv');
staging_to_out = readmatrix('/home/williamsnider/Code/libfranka/MATLAB/planned_paths/Slot5_staging_to_out_70%.csv');

% Append fake data for gripper
home_to_out = [home_to_out, zeros(size(home_to_out,1),2)]; 
out_to_above = [out_to_above, zeros(size(out_to_above,1),2)]; 
above_to_slot = [above_to_slot, zeros(size(above_to_slot,1),2)]; 
slot_to_above = [slot_to_above, zeros(size(slot_to_above,1),2)]; 
above_to_out = [above_to_out, zeros(size(above_to_out,1),2)]; 
out_to_staging = [out_to_staging, zeros(size(out_to_staging,1),2)]; 
staging_to_out = [staging_to_out, zeros(size(staging_to_out,1),2)]; 

% Create video
% addpath('/home/williamsnider/Code/libfranka/MATLAB/kinematics')
panda = loadPandaWithShape();
env = build_collision_environment();
env(end) = [];

close;
ax = show(panda, home_to_out(1,:), 'Frames', 'off', 'Visuals', 'on', 'Collisions', 'off'); 
axis off
hold on;
for i=1:numel(env)
    [~, patchObj] = show(env{i});
    patchObj.FaceColor = [0 1 1];
    patchObj.EdgeColor = 'none';
end


h = light;
h.Style = 'infinite';
h.Position = [-10, 1, 1];

plotJointMotion(panda, home_to_out, env)
plotJointMotion(panda, out_to_above, env)
plotJointMotion(panda, above_to_slot, env)
pause(1)
plotJointMotion(panda, slot_to_above, env)
plotJointMotion(panda, above_to_out, env)
plotJointMotion(panda, out_to_staging, env)
pause(1)
plotJointMotion(panda, staging_to_out, env)
plotJointMotion(panda, out_to_above, env)
plotJointMotion(panda, above_to_slot, env)
pause(1)
plotJointMotion(panda, slot_to_above, env)
plotJointMotion(panda, above_to_out, env)
plotJointMotion(panda, out_to_home, env)

h = light;
h.Style = 'infinite';
h.Position = [-1, 0, 0];

lighting gouraud
title('Picking Object')
rateControlledJointMotion(panda, ax, anglesFromStaging)
rateControlledJointMotion(panda, ax, anglesToSlot)
rateControlledJointMotion(panda, ax, anglesFromSlot)
title('Moving to Staging')
rateControlledJointMotion(panda, ax, anglesToStaging)
title('Present to Monkey')
rateControlledJointMotion(panda, ax, anglesStagingToMonkey)
title('Hold for Monkey (2s)')
rateControlledJointMotion(panda, ax, anglesHold)
title('Pull from Monkey (3s)')
rateControlledJointMotion(panda, ax, anglesMonkeyToPull)
title('Return to Staging')
rateControlledJointMotion(panda, ax, anglesPullToStaging)
title('Returning Object')
rateControlledJointMotion(panda, ax, anglesFromStaging)
rateControlledJointMotion(panda, ax, anglesToSlot)
rateControlledJointMotion(panda, ax, anglesFromSlot)



