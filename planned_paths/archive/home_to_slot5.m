clear; close all;
run /home/williamsnider/Code/libfranka/MATLAB/add_all_paths.m
run /home/williamsnider/Code/libfranka/MATLAB/parameters.m

panda = loadPandaWithShape();
env = build_collision_environment();

%% Inputs
ABOVE_DIST = 0.025;
OUT_DIST = 0.2;
% xyz = [0.6500649, 0.47182958, 0.554351];
% theta = atan2(xyz(2), xyz(1));
% % T = eul2tform([theta,-pi/2,pi]); % 180 deg rotation of end effector
% T = eul2tform([theta,pi/2,0]); 
% T(1:3,4) = xyz;
window_size = 50;  % Affects smoothing of cartesian path (for jerk)
num_waypoints = 10;  % Affects how densely the cartesian path must be linear
% q_slot5 = [1.59786,1.05315,-1.50036,-1.39843,2.52167,2.73922,2.46276, 0.01, 0.01];
q_slot5 = [1.58885,0.982282,-1.43603,-1.38394,2.60255,2.78287,2.28877, 0.01, 0.01];
%% Common
ik = inverseKinematics('RigidBodyTree', panda);
weights = [1 1 1 1 1 1];

%% Poses

pose_slot = getTransform(panda, q_slot5,'panda_hand_tcp' );

pose_above = pose_slot;
pose_above(3,4) = pose_above(3,4)+ABOVE_DIST;

vec = pose_above(1:2,4);
vec_norm = vec / norm(vec);
pose_out = pose_above;
pose_out(1:2, 4) = pose_out(1:2,4) - vec*OUT_DIST;

poseArray = cat(3,pose_slot, pose_above, pose_out);

%% Cartesian paths
[paths, all_valid] = calc_sequence_cartesian_paths(panda, poseArray,q_slot5,window_size, num_waypoints, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute, jointMin, jointMax);

slot_to_above = paths{1};
above_to_out = paths{2};

% Reverse paths
out_to_above = flip(above_to_out, 1);
above_to_slot = flip(slot_to_above,1);

% Check
assert(all(abs(slot_to_above(1,1:7)-q_slot5(1:7))<1e-5))

%% Path to home
home_to_out = planJointToJoint(panda, env, q_home,above_to_out(end,:), vMaxAll, aMaxAll, jMaxAll);

% Reverse path
out_to_home = flip(home_to_out,1);

%% Path to staging
out_to_staging = planJointToJoint(panda, env, above_to_out(end,:), q_staging, vMaxAll, aMaxAll, jMaxAll);

% Reverse path
staging_to_out = flip(out_to_staging,1);

%% Present to monkey
% Inputs
% MOVE_TIME = 0.5; % s
% stagingPose = getTransform(panda, q_staging, 'panda_hand_tcp');
% monkeyPose = stagingPose;
% monkeyPose(1:3, 4) = monkeyXYZ;
% T = calcCartesianPoseMotion(stagingPose, monkeyPose, MOVE_TIME, vMaxCartTrans, aMaxCartTrans, jMaxCartTrans, vMaxCartRot, aMaxCartRot, jMaxCartRot);
% staging_to_monkey = reshape(T,16,[])';

% Reverse path
% monkey_to_staging = reshape(T,16,[])';

%% Hold for monkey
% HOLD_TIME = 1.0;
% hold_monkey = repmat(reshape(monkeyPose, 1,16), ceil(HOLD_TIME*1000), 1);


%% Visualize
combined_cell = {home_to_out; out_to_above; above_to_slot; slot_to_above; above_to_out; out_to_staging; staging_to_out; out_to_above; above_to_slot;slot_to_above; above_to_out; out_to_home};
combined = [];
for i=1:numel(combined_cell)
    combined = [combined; combined_cell{i}];
end
plotJointMotion(panda, combined, env);
% plotPoseMotion(panda, T, {}, q_staging);

%% Write to csv
slot_name = 'Slot5';
speed_factor = num2str(vScale*100);

% Joint positions
writematrix(home_to_out(:, 1:7), [slot_name,'_home_to_out_',speed_factor,'%.csv'])
writematrix(out_to_above(:, 1:7), [slot_name,'_out_to_above_',speed_factor,'%.csv'])
writematrix(above_to_slot(:, 1:7), [slot_name,'_above_to_slot_',speed_factor,'%.csv'])
writematrix(slot_to_above(:, 1:7), [slot_name,'_slot_to_above_',speed_factor,'%.csv'])
writematrix(above_to_out(:, 1:7), [slot_name,'_above_to_out_',speed_factor,'%.csv'])
writematrix(out_to_staging(:, 1:7), [slot_name,'_out_to_staging_',speed_factor,'%.csv'])
writematrix(staging_to_out(:, 1:7), [slot_name,'_staging_to_out_',speed_factor,'%.csv'])
writematrix(out_to_home(:, 1:7), [slot_name,'_out_to_home_',speed_factor,'%.csv'])

% % Poses
% writematrix(staging_to_monkey, [slot_name,'_staging_to_monkey_',speed_factor,'%.csv'])
% writematrix(hold_monkey, [slot_name,'_hold_monkey_',speed_factor,'%.csv'])
% writematrix(monkey_to_staging, [slot_name,'_monkey_to_staging_',speed_factor,'%.csv'])


%% Checks
plot_derivatives(combined(:,1:7))
plotJointScaled(combined(:,1:7), jointMax, jointMin)

for i=1:numel(combined_cell)
    array = combined_cell{i};
    assert(checkTrajectory(array, array(1,:), array(end,:), vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute))
end
checkTrajectory(combined, combined(1,:), combined(end,:), vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute)
% writematrix(home_to_out(:, 1:7), 'Slot5_home_to_out_5%.csv')
% 
% show(panda, q_home, 'Collisions', 'on');
% hold on;
% for i=1:numel(env)
%     show(env{i})
% end
% 
% 
% % Calc path to monkey
% % out_to_staging = planJointToJoint(panda, {}, above_to_out(end,:), stagingAngles, vMaxAll, aMaxAll, jMaxAll);
% 
% % Flip reverse
% % staging_to_out = flip(out_to_staging,1);
% out_to_above = flip(above_to_out,1);
% 
% % above_to_slot = flip(slot_to_above,1);
% % 
% 
% % %% Visualize
% % 
% combined = [home_to_out; out_to_above];
% plotJointScaled(q(:,1:7), jointMax, jointMin);
% plotJointMotion(panda, combined, {})
% % plotPoseMotion(panda, T, {}, stagingAngles);
% % pause(0.5)
% % plotPoseMotion(panda, flip(T,3), {}, stagingAngles);
% % plotJointMotion(panda, flip(combined,1), {})
% % 
% % %% Export as csv
% % writematrix(slot_to_above, 'Slot5_slot_to_above.csv')
% % writematrix(above_to_out, 'Slot5_above_to_out.csv')
% % writematrix(out_to_staging, 'Slot5_out_to_staging.csv')
% % writematrix(staging_to_out, 'Slot5_staging_to_out.csv')
% writematrix(out_to_above, 'Slot5_out_to_above_5%.csv')
% % writematrix(above_to_slot, 'Slot5_above_to_slot.csv')
% % 
% % 
% % 
% % 
% % 
% % 
% % % % 
% % % plot_derivatives(combined)
% % % 
% % % checkTrajectory(combined, combined(1,:), combined(end,:), vMaxAll, aMaxAll, jMaxAll)
% % % 
% % % %TODO: Figure out why above_to_out violates acceleration limits?
% % % plot_derivatives(above_to_out)
% % % % checkTrajectory(above_to_out, above_to_out(1,:), above_to_out(end,:), vMaxAll, aMaxAll, jMaxAll)
% % % 
% % % 
% % % checkTrajectory(combined, combined(1,:), combined(end,:), vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute)
% % % 
% % % q = above_to_out(:,end)';
