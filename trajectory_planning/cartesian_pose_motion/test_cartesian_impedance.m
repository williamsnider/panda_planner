clear; close all;
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/add_all_paths.m
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/parameters.m

% Inputs
MOVE_TIME = 2.5; % s


panda = loadPandaWithShape();
stagingPose = getTransform(panda, q_staging, 'panda_hand_tcp');
monkeyPose = stagingPose;
monkeyPose(1:3, 4) = monkeyXYZ;
T = calcCartesianPoseMotion(stagingPose, monkeyPose, MOVE_TIME, vMaxCartTrans, aMaxCartTrans, jMaxCartTrans, vMaxCartRot, aMaxCartRot, jMaxCartRot);

%% Visualize
plotPoseMotion(panda, T, {}, q_staging);

