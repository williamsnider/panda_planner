% clear; close all;
% run /home/oconnorlabmatlab/Code/libfranka/MATLAB/add_all_paths.m
% run /home/oconnorlabmatlab/Code/libfranka/MATLAB/parameters.m
% 
% % Inputs
% ABOVE_DIST = 0.025;
% OUT_DIST = 0.15;
% % q_slot = [-1.24666179, 0.69630698,  2.02356136,-2.13780334,  -2.62650107,  2.73801965, -0.29689215, 0.01, 0.01];
% q_slot = [1.58899,0.985033,-1.44443,-1.39649,2.60352,2.74305,2.31824, 0.01, 0.01];
% panda = loadPandaWithShape();
% 
% % Calc pose of waypoints
% pose_slot = getTransform(panda, q_slot, 'panda_hand_tcp');
% 
% pose_slot_above = pose_slot;
% pose_slot_above(3,4) = pose_slot_above(3,4)+ABOVE_DIST;
% 
% vec = pose_slot_above(1:2,4);
% vec_norm = vec / norm(vec);
% pose_slot_out = pose_slot_above;
% pose_slot_out(1:2, 4) = pose_slot_out(1:2,4) - vec*OUT_DIST;
% 
% % Calc path between waypoints
% window_size = 50;
% num_waypoints = 4;
% [slot_to_above, con_q, con_t, T] = calc_cartesian_path(panda, pose_slot, pose_slot_above,q_slot, window_size, num_waypoints, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute);
% [above_to_out, con_q, con_t, T] = calc_cartesian_path(panda, pose_slot_above, pose_slot_out,slot_to_above(end,:), window_size,  num_waypoints, vMaxAll, aMaxAll, jMaxAll, vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute);
% 
% % Calc path to monkey
% out_to_staging = planJointToJoint(panda, {}, above_to_out(end,:), stagingAngles, vMaxAll, aMaxAll, jMaxAll);
% 
% % Flip reverse
% staging_to_out = flip(out_to_staging,1);
% out_to_above = flip(above_to_out,1);
% above_to_slot = flip(slot_to_above,1);
% 
% % Present to monkey
% % Inputs
% MOVE_TIME = 0.5; % s
% stagingPose = getTransform(panda, stagingAngles, 'panda_hand_tcp');
% monkeyPose = stagingPose;
% monkeyPose(1:3, 4) = monkeyXYZ;
% T = calcCartesianPoseMotion(stagingPose, monkeyPose, MOVE_TIME, vMaxCartTrans, aMaxCartTrans, jMaxCartTrans, vMaxCartRot, aMaxCartRot, jMaxCartRot);
% 
% %% Visualize
% 
% combined = [slot_to_above; above_to_out; out_to_staging];
% plotJointMotion(panda, combined, {})
% plotPoseMotion(panda, T, {}, stagingAngles);
% pause(0.5)
% plotPoseMotion(panda, flip(T,3), {}, stagingAngles);
% plotJointMotion(panda, flip(combined,1), {})
% 
% %% Export as csv
% writematrix(slot_to_above, 'Slot5_slot_to_above.csv')
% writematrix(above_to_out, 'Slot5_above_to_out.csv')
% writematrix(out_to_staging, 'Slot5_out_to_staging.csv')
% writematrix(staging_to_out, 'Slot5_staging_to_out.csv')
% writematrix(out_to_above, 'Slot5_out_to_above.csv')
% writematrix(above_to_slot, 'Slot5_above_to_slot.csv')
% 
% 
% 
% 
% 
% 
% % % 
% % plot_derivatives(combined)
% % 
% % checkTrajectory(combined, combined(1,:), combined(end,:), vMaxAll, aMaxAll, jMaxAll)
% % 
% % %TODO: Figure out why above_to_out violates acceleration limits?
% % plot_derivatives(above_to_out)
% % % checkTrajectory(above_to_out, above_to_out(1,:), above_to_out(end,:), vMaxAll, aMaxAll, jMaxAll)
% % 
% % 
% % checkTrajectory(combined, combined(1,:), combined(end,:), vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute)
% % 
% % q = above_to_out(:,end)';
