% Code to find the best configurations for the staging area / monkey grasp
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Set random stream for reproducibility
stream = RandStream('mt19937ar', 'Seed', 123); 
RandStream.setGlobalStream(stream);

%% Inputs
shape_offset = 0.07;
TRAVEL_DIST = 0.15;
NUM_ATTEMPTS = 3000;
JOINT_REDUCTION = 0.3;
J7_REDUCTION = 0.2;
SAVE_DIR = "./elbow_tables/20240424_";

%% Construct list of orientations
% Ori A - pointing left, peg towards floor
ori_A = eye(3);
ori_A(1:3,1) = [0,0,-1];
ori_A(1:3,2) = [-1,0,0];
ori_A(1:3,3) = [0,1,0];


% Ori B - pointing up, 
th0 = pi/2;
R0 = [1 0 0; 0 cos(th0) -sin(th0);  0 sin(th0) cos(th0)];
ori_B = R0*ori_A;

% Ori C - pointing towards monkey, peg down
th1 = pi/2;
R1 = [cos(th1) -sin(th1) 0; sin(th1) cos(th1) 0; 0 0 1];
ori_C = R1 * ori_A;

% Ori X - middle of A and B
th2 = pi/4;
R2 = [1 0 0; 0 cos(th2) -sin(th2);  0 sin(th2) cos(th2)];
ori_X = R2 * ori_A;

% Ori Y - middle of A and C
th3 = pi/4;
R3 = [cos(th3) -sin(th3) 0; sin(th3) cos(th3) 0; 0 0 1];
ori_Y = R3 * ori_A;

% Ori Z - middle of B and C
th4 = pi/4;
R4 = [cos(th4) 0 sin(th4); 0 1 0; -sin(th4) 0 cos(th4)];
ori_Z = R4 * ori_C;

% Ori D - middle of A,B,C
ori_D = R4 * ori_Y;

ori_cell = {ori_A, ori_B, ori_C, ori_D, ori_X, ori_Y, ori_Z};
ori_names = {"ori_A", "ori_B", "ori_C", "ori_D", "ori_X", "ori_Y", "ori_Z"};

plot_ori(ori_cell)


%% Load robot, restrict joint limits for safety
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();

% Restrict joints 1-6 by JOINT_REDUCTION
for body_num = 1:6
    oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
    newLimits = oldLimits + [JOINT_REDUCTION,-JOINT_REDUCTION];
    panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;
end

% Restrict joint 7 by J7_REDUCTION
body_num = 7;
oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
newLimits = oldLimits + [J7_REDUCTION,-J7_REDUCTION];
panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;

% Construct ik solver
ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
weights = [1 1 1 1 1 1];

%% Test XYZ positions for extreme configuration to see which is valie
% 
% XYZ_arr = [];
% for X = -0.80:0.025:-0.60
%     for Y = 0.0:0.025:0.2
%         for Z = (10:22)*0.0254
%             XYZ_arr = [XYZ_arr;X,Y,Z];
%         end
%     end
% end
% 
% 
% % Find valid q's using ik
% parfor XYZ_num = 1:size(XYZ_arr,1)
%      warning('off', 'all');
% %    disp(XYZ_num)
%     XYZ = XYZ_arr(XYZ_num,:);
% 
%     T_cell = {};
%     for cell_num = 1:numel(ori_cell)
%         T_cell{cell_num} = construct_pose(ori_cell{cell_num}, XYZ', shape_offset);
%     end
% 
%     T_success = zeros(1, numel(T_cell));
%     for cell_num = 1:numel(T_cell)
% 
% 
%         % Test if XYZ position is valid
%         attempt_num = 0;
%         while attempt_num < NUM_ATTEMPTS
%             attempt_num = attempt_num + 1;
%             T_extreme = T_cell{cell_num};
%             [success, q_extreme] = find_valid_staging_pose(panda_sc,ik, T_extreme, TRAVEL_DIST, J7_CUTOFF);
%             if success
%                 T_success(cell_num) = true;
%                 break
%             end
%         end
% 
%         % Exit early if one orientation failed
%         if all(T_success(1:cell_num)) == false
%             break
%         end
% 
%     end
% 
%     if all(T_success)
%         disp(XYZ)
%     end
% 
% end

%% Choose best XYZ, use to calculate staging and extreme configurations

XYZ = [-0.6250 0.1750 0.5588];   % Best XYZ given the orientations we chose


T_cell = {};
for cell_num = 1:numel(ori_cell)
    T_cell{cell_num} = construct_pose(ori_cell{cell_num}, XYZ', shape_offset);
end

success_cell = {};
for cell_num = 1:numel(ori_cell)
    success_cell{cell_num} = [];
end

disp("Testing if XYZ is valid")
for cell_num = 1:numel(T_cell)
    cell_num
 

    % Test if XYZ position is valid (can find q_extreme and q_staging)
    attempt_num = 0;
    while attempt_num < NUM_ATTEMPTS
        attempt_num = attempt_num + 1;
        T_extreme = T_cell{cell_num};
        [success, q_extreme, q_staging_temp] = find_valid_staging_pose(panda_sc,ik, T_extreme, TRAVEL_DIST, J7_REDUCTION);
        if success
            T_success(cell_num) = true;
            success_cell{cell_num} = [success_cell{cell_num}; [q_extreme, q_staging_temp]];
        end

        if size(success_cell{cell_num},1) > 100
            break
        end
    end

end

%% Choose q_extreme that is furthest from joint limits; recalculate q_staging

for cell_num = 1:numel(ori_names)

    num_configurations = size(success_cell{cell_num},1);
    scaled_dist_to_edge_arr = zeros(num_configurations,1);
    
    for i = 1:num_configurations
    
        % Get staging and extreme positions
        q_e = success_cell{cell_num}(i,1:9);
        
        q = q_e(1:7);
        scaled_positions = (q-params.jointMin)./(params.jointMax-params.jointMin);
        scaled_positions = scaled_positions(1:6);
        q_dist_e = min(min(scaled_positions),min(1-scaled_positions));
        scaled_dist_to_edge_arr(i) = q_dist_e;
    end
    
    % Chose best q_dist
    [~, idx] = max(scaled_dist_to_edge_arr);
    
    % Calculate elbow table and q_staging
    q_extreme = success_cell{cell_num}{idx,1:9};
    poseLetter = ori_names{cell_num}{1}(5);
    [elbow_table, q_staging] = calc_elbow_table(q_extreme, panda_sc, TRAVEL_DIST, ik, SAVE_DIR, letter);
    
    % Display q_staging and q_extreme at different joint values
    for rot_num = 0:3
    
        % Staging
        poseName = "staging" + poseLetter + num2str(rot_num);
        q_s = q_staging;
        q_s(7) = q_s(7) + pi/2*rot_num;
        if q_s(7) > 2.8
            q_s(7) = q_s(7)-2*pi;
        end
        msg = poseName + " = [" + num2str(q_s)+ "];";
        disp(msg)
    
        % Extreme
        poseName = "extreme" + poseLetter + num2str(rot_num);
        q_e = q_extreme;
        q_e(7) = q_e(7) + pi/2*rot_num;
        if q_e(7) > 2.8
            q_e(7) = q_e(7)-2*pi;
        end
        msg = poseName + " = [" + num2str(q_e)+ "];";
        disp(msg)
    
    
    end
end
