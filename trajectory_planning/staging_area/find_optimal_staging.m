% Code to find the best configurations for the staging area / monkey grasp

clear; close all;
addpath("../..")
params = CustomParameters();

%% Inputs
SAVE_DIR = "ballpark/";
Y = 0.15;
Z = 19*0.0254;
OFFSET = 0.052;
TRAVEL_DIST = 0.15;


%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();

ss = ManipulatorStateSpaceSphere(panda_ec, panda_sc);
sv = ManipulatorStateValidatorSphere(ss, env, params.validationDistance, params);
sv.IgnoreSelfCollision = false;
sv.Environment = env;

%% Adjust joint limits for panda_sc

% disp("DECREASING JOINT LIMITS FOR PANDA_SC ONLY")
% thres = 0.10;
% for body_num = 1:panda_sc.NumBodies
% 
%     joint = panda_sc.Bodies{body_num}.Joint;
%     
% 
%     % Skip non revolute
%     if ~strcmp(joint.Type ,'revolute')
%         continue
%     end
% 
%     % Update values to be 10% of max
%     old = joint.PositionLimits;
% 
%     range = old(2)-old(1);
%     new_min = old(1) + range*(thres);
%     new_max = old(1) + range*(1-thres);
%     new = [new_min, new_max];
% 
%     panda_sc.Bodies{body_num}.Joint.PositionLimits = new;
% 
% end



%% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 500;
weights = [1 1 1 1 1 1];
ss = manipulatorStateSpace(panda_ec); % for joint limits
stateBounds = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits


%% Construct list of orientation
pose_map = containers.Map('KeyType','char','ValueType','any');

% Ori A - pointing right, peg towards floor
ori_A = eye(3);
ori_A(1:3,1) = [0,0,-1];
ori_A(1:3,2) = [1,0,0];
ori_A(1:3,3) = [0,-1,0];


% Ori B - pointing down, peg towards robot
ori_B = eye(3);
ori_B(1:3,1) = [1,0,0];
ori_B(1:3,2) = [0,-1,0];
ori_B(1:3,3) = [0,0,-1];

% Ori C - pointing left, peg towards floor
ori_C = eye(3);
ori_C(1:3,1) = [0,0,-1];
ori_C(1:3,2) = [-1,0,0];
ori_C(1:3,3) = [0,1,0];

% Ori D - pointing up, peg towards robot
ori_D = eye(3);
ori_D(1:3,1) = [1,0,0];
ori_D(1:3,2) = [0,1,0];
ori_D(1:3,3) = [0,0,1];

% Ori E - pointing towards monkey, peg towards floor
ori_E = eye(3);
ori_E(1:3,1) = [0,0,-1];
ori_E(1:3,2) = [0,-1,0];
ori_E(1:3,3) = [-1,0,0];

ori_cell = {ori_A, ori_B, ori_C, ori_D, ori_E};


% Calculate a cartesian path between two extremes
X_list = -0.70:0.01:-0.60;
q_list_best_cell = cell(numel(X_list), numel(ori_cell));
edge_dist_cell = cell(numel(X_list), numel(ori_cell));

ori = ori_cell{1};
X = -0.70;
XYZ = [X;Y;Z];
T0 = construct_pose(ori, XYZ, OFFSET);
TF = T0;
TF(1,4) = TF(1,4)+TRAVEL_DIST;

dist_from_edge_best = 0;
for i=1:100
initialGuess = randomConfiguration(panda_sc);
[q,solnInfo] = ik('panda_hand_tcp',T0,[1 1 1 1 1 1],initialGuess);
scaled_list = calc_scaled(q, params);
dist_from_edge = min(min(1-scaled_list(1:6)), min(scaled_list(1:6)));

if dist_from_edge > dist_from_edge_best
    disp(q)
    dist_from_edge_best = dist_from_edge;
    disp(dist_from_edge_best)
end
end

%% Plot robot
show(panda_sc, q)
% plotJointMotion(panda_sc, q, env, params)
plot3(params.shelf_pts(:,1),params.shelf_pts(:,2),params.shelf_pts(:,3), "r*")

% 
% % Find q that matches TF
% for loop_count = 1:200
%     initialGuess = randomConfiguration(panda_sc);
%     [q_initial,solnInfo] = ik('panda_hand_tcp',tforms(:,:,1),[1 1 1 1 1 1],initialGuess);
%     if strcmp(solnInfo.Status, "best available")
%         continue
%     end
%     q_list = [q_initial];
% end
% 
% 
% for ori_num = 1:numel(ori_cell)
%     ori = ori_cell{ori_num};
%     for X_num = 1:numel(X_list)
%         disp(X_num)
%         X = X_list(X_num);
% 
%         XYZ = [X;Y;Z];
%         T0 = construct_pose(ori, XYZ, OFFSET);
% 
%         [dist_from_edge_best, q_list_best] = find_best_q_for_cartesian_path(T0, TRAVEL_DIST, panda_sc, ik, params);
% 
%         edge_dist_cell{X_num, ori_num} = dist_from_edge_best;
%         q_list_best_cell{X_num, ori_num} = q_list_best;
%     end
% 
% end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Choose X = -0.65 based on edge_dist_cell
% X = -0.65;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % Create poses
% msg_array = [];
% for ori_num =1:numel(ori_cell)
%     
%     % Get pose
%     ori = ori_cell{ori_num};
%     XYZ = [X;Y;Z];
%     T0 = construct_pose(ori, XYZ, OFFSET);
% 
%     % Calculate q that gives acceptable cartesian path
%     [dist_from_edge_best, q_list_best] = find_best_q_for_cartesian_path(T0, TRAVEL_DIST, panda_sc,ik, params);
%     assert(dist_from_edge_best>0.05);
%         
%     % Adjust q's 7th joint for different rotations
%     q_staging = q_list_best(end,:);
%     j7 = q_staging(7);
%     jValsStaging = j7:pi/2:j7+3*pi/2;
%     jValsStaging(jValsStaging > params.jointMax(7)) = jValsStaging(jValsStaging >params.jointMax(7)) - 2*pi;      % roll jValsStaging to be within joint limits
% 
%     % Adjust q_extreme's 7th joint for diferent rotations;
%     q_extreme = q_list_best(1,:);  % Most extreme position (monkey should be prevented from pulling further)
%     j7 = q_extreme(7);
%     jValsExtreme = j7:pi/2:j7+3*pi/2;
%     jValsExtreme(jValsExtreme > params.jointMax(7)) = jValsExtreme(jValsExtreme > params.jointMax(7)) - 2*pi;
% 
%     for rot_num = 0:3
% 
%         % Update q_staging
%         q_staging_update = q_staging;
%         q_staging_update(7) = jValsStaging(rot_num+1);
%         q_staging_update(8:9) = 0.01;  
% 
%         % Pose Name
%         letters = 'ABCDE';
%         poseLetter = letters(ori_num);
%         poseName = "staging_" + poseLetter + num2str(rot_num);
% 
%         % Print result
%         msg = poseName + " = [" + num2str(q_staging_update)+ "]";
%         disp(msg)
%         msg_array = [msg_array;msg];
% 
%         % Update q_extreme
%         q_extreme_update = q_extreme;
%         q_extreme_update(7) = jValsStaging(rot_num+1);
%         q_extreme_update(8:9) = 0.01;  
% 
%         % Pose Name
%         letters = 'ABCDE';
%         poseLetter = letters(ori_num);
%         poseName = "extreme_" + poseLetter + num2str(rot_num);
% 
%         % Print result
%         msg = poseName + " = [" + num2str(q_extreme_update)+ "]";
%         disp(msg)
%         msg_array = [msg_array;msg];
%     end
% end
% 
% 
% % Display msg
% for row_num = 1:size(msg_array,1)
%     msg = msg_array(row_num,:);
%     disp(msg)
% end
% 
% % Evaluate msg
% for row_num = 1:size(msg_array,1)
%     msg = msg_array(row_num,:);
%     eval(msg)
% end
% 
% 
% 
% 
% %% Plot robot
% plotJointMotion(panda_sc, q, env, params)
% plot3(params.shelf_pts(:,1),params.shelf_pts(:,2),params.shelf_pts(:,3), "r*")



function [dist_from_edge_best, q_list_best] = find_best_q_for_cartesian_path(T0, TRAVEL_DIST, panda_sc,ik, params)

        TF = T0;
        TF(1,4) = TF(1,4)+TRAVEL_DIST;
        nSamples = 5;
        tSamples = linspace(0,1,nSamples);
        tInterval = [0,1];
        [tforms,~,~] = transformtraj(T0,TF,tInterval,tSamples);
    
        dist_from_edge_best = 0;
        q_list_best = [];
        for loop_count = 1:200
            initialGuess = randomConfiguration(panda_sc);
            [q_initial,solnInfo] = ik('panda_hand_tcp',tforms(:,:,1),[1 1 1 1 1 1],initialGuess);
            if strcmp(solnInfo.Status, "best available")
                continue
            end
            q_list = [q_initial];
            scaled_list = calc_scaled(q_initial, params);
            for sample_num = 2:nSamples
                [q,solnInfo] = ik('panda_hand_tcp',tforms(:,:,sample_num),[1,1,1,1,1,1],q_list(sample_num-1,:));
                q_list = [q_list;q];
                scaled_list = [scaled_list; calc_scaled(q, params)];
            end
            dist_from_upper = 1-max(max(scaled_list));
            dist_from_lower = min(min(scaled_list));
            dist_from_edge = min(dist_from_upper, dist_from_lower);
    
            % Confirm jVals not to close to min/max
            j7 = q_list(1,7);
            jVals = j7:pi/2:j7+3*pi/2;
            jVals(jVals > params.jointMax(7)) = jVals(jVals >params.jointMax(7)) - 2*pi;
            if any(abs(jVals-params.jointMax(7))< 0.1)
                dist_from_edge=0;
            elseif any(any(abs(jVals-params.jointMin(7))< 0.1))
                dist_from_edge=0;
            end

            j7 = q_list(end,7);
            jVals = j7:pi/2:j7+3*pi/2;
            jVals(jVals > params.jointMax(7)) = jVals(jVals >params.jointMax(7)) - 2*pi;
            if any(abs(jVals-params.jointMax(7))< 0.1)
                dist_from_edge=0;
            elseif any(any(abs(jVals-params.jointMin(7))< 0.1))
                dist_from_edge=0;
            end

            if dist_from_edge>dist_from_edge_best
                dist_from_edge_best = dist_from_edge;
                q_list_best =q_list;
            end
        end

end

