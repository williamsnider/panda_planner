clear; close all;
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/add_all_paths.m
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/parameters.m

panda = loadPandaWithShape();
env = build_collision_environment();

%% Hold for staging
HOLD_TIME = 1.0;
stagingPose = getTransform(panda, q_staging, 'panda_hand_tcp');
num_steps = ceil(HOLD_TIME*1000);
hold_staging = repmat(reshape(stagingPose, 1,16), num_steps, 1);
writematrix(hold_staging, ['hold_staging_',num2str(num_steps),'.csv'])

%% Hold for home 
HOLD_TIME = 1.0;
homePose = getTransform(panda, q_home, 'panda_hand_tcp');
num_steps = ceil(HOLD_TIME*1000);
hold_home = repmat(reshape(homePose, 1,16), num_steps, 1);
writematrix(hold_home, ['hold_home_',num2str(num_steps),'.csv'])

%% Home slight move
MOVE_TIME = 2;
goalPose = homePose;
goalPose(15) = goalPose(15)+0.05;
T = calcCartesianPoseMotion(homePose, goalPose, MOVE_TIME, vMaxCartTrans, aMaxCartTrans, jMaxCartTrans, vMaxCartRot, aMaxCartRot, jMaxCartRot);
home_to_goal = reshape(T,16,[])';
goal_to_home = flip(home_to_goal, 1);
num_steps = size(home_to_goal, 1);
writematrix(home_to_goal, ['home_to_goal_',num2str(num_steps),'.csv'])
writematrix(goal_to_home, ['goal_to_home_',num2str(num_steps),'.csv'])


%% Home to Staging
home_to_staging = planJointToJoint(panda, env, q_home, q_staging, vMaxAll, aMaxAll, jMaxAll);
staging_to_home = flip(home_to_staging,1);
% plotJointMotion(panda, [home_to_staging;staging_to_home], env)
speed_factor = num2str(ceil(vScale*100));
writematrix(home_to_staging(:, 1:7), ['home_to_staging_',speed_factor,'%.csv'])
writematrix(staging_to_home(:, 1:7), ['staging_to_home_',speed_factor,'%.csv'])

