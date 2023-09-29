% Find the optimal shelf dimension (radius and height) to maximize ability
% to store shapes.


%% Load robot
clear; close all;
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/add_all_paths.m
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/parameters.m
% delete(gcp('nocreate'))
% parpool()

panda = loadPandaWithShape();
env = build_collision_environment();

%% Clear folder where positions are saved
basefolder = pwd+"/saved_shape_positions/";
% file_list = dir(basefolder);
% for i = 1:numel(file_list)
%     filename = file_list(i).folder+"/"+file_list(i).name;
%     if contains(filename,".csv")
%         delete(filename)    
%     end
% end

%% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree',panda);
ik.SolverParameters.MaxIterations = 10000;
weights = [1 1 1 1 1 1];

ss = manipulatorStateSpace(panda);

%% Inputs
num_theta_samples = 9;

z_samples = -0.2:0.1:0.9;
num_z_samples = numel(z_samples);

radius_samples = 1:-0.01:0.25; % note descending
num_radius_samples = numel(radius_samples);

theta_samples = linspace(-3*pi/4, 3*pi/4, num_theta_samples);


%% Debug
z_idx = 7;
radius_idx = 25;
theta_idx = 5;

z = z_samples(z_idx);
radius = radius_samples(radius_idx);
theta = theta_samples(theta_idx);

x = cos(theta) * radius;
y = sin(theta) * radius;
% T = eul2tform([0, pi/2,-theta]);
T = eul2tform([theta, 3*pi/2, pi]);

T(1:3, 4) = [x,y,z];
% valid=false;
% while ~valid
%     [q,solnInfo] = ik('panda_hand_tcp',T,weights,randomConfiguration(panda));
%     valid = isStateValid(sv,q);
% end
% 
% % solnInfo
% q(7)
% isStateValid(sv,q)
% Tf = getTransform(panda, q, 'panda_hand_tcp');
% ax=show(panda, q, 'Visuals', 'on', 'Frames','on', 'Collisions', 'on'); hold on;
% %Show remaining objects
% for i = 1:numel(env)
%     [~, patchObj] = show(env{i}, "Parent", ax);
%     patchObj.FaceColor = [0 1 1];
%     patchObj.EdgeColor = 'none';
% end

%% Calc joint angles
found_valid_cartesian_path = false;
q_list=[];
for i = 1:50
    z
    initialGuess = randomConfiguration(panda);
    [q,solnInfo] = ik('panda_hand_tcp',T,weights,initialGuess);
    
    % Exit loop if inverse kinematics fails
    if ~strcmp(solnInfo.Status, "success")
        disp('Inverse kinematics failed.')
        ik.SolverParameters.EnforceJointLimits=false;
        [new_q,solnInfo] = ik('panda_hand_tcp',T,weights,q);
        break
    end

    %Redo inverse kinematics if q is outside joint limits or in collision
    q = round(q, 10);  % Rounding ensures not exceeding joint limits by precision error
    if ~custom_check_valid_state(panda, env, q, ss.StateBounds')
        disp("Initial state not valid")
        continue
    end

    %% Calc poses for slot->above->out motion
    ABOVE_HEIGHT = 0.025;
    OUT_DIST = 0.1;
    pose_slot = getTransform(panda, q,'panda_hand_tcp' );
    pose_above = pose_slot;
    pose_above(3,4) = pose_above(3,4)+ABOVE_HEIGHT;

    vec = pose_above(1:2,4);
    vec_norm = vec / norm(vec);
    pose_out = pose_above;
    pose_out(1:2, 4) = pose_out(1:2,4) - vec*OUT_DIST;

    poseArray = cat(3,pose_slot, pose_above, pose_out);

    %% Cartesian paths
    window_size = 50;  % Affects smoothing of cartesian path (for jerk)
    num_waypoints = 10;  % Affects how densely the cartesian path must be linear
    [all_paths, all_valid] = calc_sequence_cartesian_paths(panda, env, ss.StateBounds', poseArray,q,window_size, num_waypoints, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute, jointMin, jointMax);

    if all_valid==true
        found_valid_cartesian_path = true;
        combined = [all_paths{1}; all_paths{2}];
        break;
    end
end

if found_valid_cartesian_path==true
    disp("Valid found");
    %     combined = [all_paths{1}; all_paths{2}];
    %     plotJointMotion(panda, combined, env)
    q_list = [q_list;q];
else
    %     show(panda,q,'Collisions', 'off')
    disp("No valid cartesian path found")
end

xyz_list = [];
T_out = getTransform(panda, q, 'panda_hand_tcp');
xyz = T_out(1:3,4)';
xyz_list = [xyz_list;xyz];

ax=show(panda, q, 'Visuals', 'on', 'Frames','on', 'Collisions', 'off'); hold on;
plot3(xyz_list(:,1), xyz_list(:,2), xyz_list(:,3), 'r*');hold on;

%Show remaining objects
for i = 1:numel(env)
    [~, patchObj] = show(env{i}, "Parent", ax);
    patchObj.FaceColor = [0 1 1];
    patchObj.EdgeColor = 'none';
end




% %% Real code
% % parfor z_idx = 1:num_z_samples
% parfor z_idx = 6:6
%     z = z_samples(z_idx)
%     q_list = [];
% 
%     for radius_idx =1:num_radius_samples
%         disp("z="+num2str(z_idx)+" radius_idx="+num2str(radius_idx))
%         radius = radius_samples(radius_idx);
% 
%         theta_violation_count = 0;
%         for theta_idx = 1:num_theta_samples
%             theta = theta_samples(theta_idx);
% 
%             %% Calculate pose
%             x = cos(theta) * radius;
%             y = sin(theta) * radius;
%             T = eul2tform([0, pi/2,-theta]);
%             T(1:3, 4) = [x,y,z];
% 
%             %% Calc joint angles
%             found_valid_cartesian_path = false;
%             for i = 1:30
% 
%                 initialguess = randomConfiguration(panda);
% 
%                 [q,solnInfo] = ik('panda_hand_tcp',T,weights,initialguess);
% 
%                 % Exit loop if inverse kinematics fails
%                 if ~strcmp(solnInfo.Status, "success")
%                     disp('Inverse kinematics failed.')
%                     break
%                 end
% 
%                 %Redo inverse kinematics if q is outside joint limits or in collision
%                 if ~isStateValid(sv, q)
%                     disp("Invalid initial state.")
%                     continue
%                 end
% 
%                 %% Calc poses for slot->above->out motion
%                 ABOVE_HEIGHT = 0.025;
%                 OUT_DIST = 0.1;
%                 pose_slot = getTransform(panda, q,'panda_hand_tcp' );
%                 pose_above = pose_slot;
%                 pose_above(3,4) = pose_above(3,4)+ABOVE_HEIGHT;
% 
%                 vec = pose_above(1:2,4);
%                 vec_norm = vec / norm(vec);
%                 pose_out = pose_above;
%                 pose_out(1:2, 4) = pose_out(1:2,4) - vec*OUT_DIST;
% 
%                 poseArray = cat(3,pose_slot, pose_above, pose_out);
% 
%                 %% Cartesian paths
%                 window_size = 50;  % Affects smoothing of cartesian path (for jerk)
%                 num_waypoints = 10;  % Affects how densely the cartesian path must be linear
%                 [all_paths, all_valid] = calc_sequence_cartesian_paths(panda, sv, poseArray,q,window_size, num_waypoints, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute, jointMin, jointMax);
% 
%                 if all_valid==true
%                     found_valid_cartesian_path = true;
%                     %                     combined = [all_paths{1}; all_paths{2}];
%                     break;
%                 end
%             end
% 
%             if found_valid_cartesian_path==true
%                 disp("Valid found");
%                 q_list = [q_list;q];
% 
%             else
%                 %     show(panda,q,'Collisions', 'off')
%                 disp("No valid cartesian path found for theta")
%                 break;
%             end
%         end
% 
%         if size(q_list,1) == num_theta_samples
%             disp("Found acceptable radius: " + num2str(radius_samples(radius_idx)))
%             if ~exist(basefolder, 'dir')
%                 mkdir(basefolder)
%             end
%             filename = basefolder+"z-"+num2str(z_idx)+"_r-"+strrep(num2str(radius_samples(radius_idx)), ".", "p")+".csv";
%             writematrix(q_list,filename);
%             break;
%         end
% 
%     end
% 
% 
% end
% 
% xyz_list = []
% file_list = dir(basefolder);
% for i = 1:numel(file_list)
%     filename = file_list(i).name;
%     if contains(filename, ".csv")
%         q_list = readmatrix(file_list(i).folder + "/" +file_list(i).name);
% 
%         for j=1:size(q_list,1)
%             q = q_list(j,:);
%             T = getTransform(panda, q, 'panda_hand_tcp');
%             xyz = T(1:3,4)';
%             xyz_list = [xyz_list;xyz];
%         end
% 
%     end
% end 
% ax=show(panda, q, 'Visuals', 'on', 'Frames','off', 'Collisions', 'off'); hold on;
% plot3(xyz_list(:,1), xyz_list(:,2), xyz_list(:,3), 'r*');hold on;
% 
% %Show remaining objects
% for i = 1:numel(env)
%     [~, patchObj] = show(env{i}, "Parent", ax);
%     patchObj.FaceColor = [0 1 1];
%     patchObj.EdgeColor = 'none';
% end
