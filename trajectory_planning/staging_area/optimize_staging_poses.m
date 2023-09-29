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

% Group data into struct
s = struct;
s.ori_A = ori_A;
s.ori_B = ori_B;
s.ori_C = ori_C;
s.ori_D = ori_D;
s.ori_E = ori_E;
s.Y = Y;
s.Z = Z;
s.OFFSET = OFFSET;
s.TRAVEL_DIST = TRAVEL_DIST;
s.panda_sc = panda_sc;
s.ik = ik;
s.params = params;

% Calculate a cartesian path between two extremes
X_list = -0.70:0.01:-0.60;
q_list_best_cell = cell(numel(X_list), numel(ori_cell));
edge_dist_cell = cell(numel(X_list), numel(ori_cell));

ori_names = {"ori_A", "ori_B", "ori_C", "ori_D", "ori_E"};
args = [];
for ori_num = 1:numel(ori_names)
    ori_Letter = ori_names{ori_num};
    for X_num = 1:numel(X_list)
        X = X_list(X_num);
        args = [args; [ori_Letter, X]];
    end

end

% Calc dists, save to .mat file
for i = 6:size(args,1)
    disp(i)
    ori_Letter = args(i, 1);
    X = str2num(args(i,2));
    calc_dist_from_edge(ori_Letter, X, s)
end

% Load all mat files in dir
directory_path = './temp'; % Current directory. Modify this if needed.
files = dir(directory_path);
fileNames = {files(~[files.isdir]).name}';% Filter out folders from the list


best = zeros(numel(X_list), 5);

% Store in table

% Print file names
for i = 1:length(fileNames)

    % Read file 
    fname = fileNames{i};
    m = load(fname);

    % Extract info from fname
    ori_Letter = fname(1:5);
    X_str= fname(7:end-4);
    X_val = str2double(X_str);

    % Get indices of X and ori_Letter for storing in "best" array
    X_idx = find(abs(X_list-X_val)<0.0001, 1);
    for ori_idx =1:5
        if strcmp(ori_names{ori_idx}, ori_Letter)
            break
        end
    end

    % Store in best array
    best_idx = find(m.dist_from_edge_arr == max(m.dist_from_edge_arr),1);
    best(X_idx, ori_idx) = m.dist_from_edge_arr(best_idx);
end

% % % %% Generate home_to_candidate staging positions
% % % % Choose X = -0.65
% % % X = -0.65;
% % % q_best = cell(1,5);
% % % mapObj = containers.Map();
% % % 
% % % 
% % % for ori_num = 1:5
% % %     ori_Letter = ori_names{ori_num};
% % %     X_str = num2str(X);
% % %     
% % %     % Load file
% % %     fname = strcat(ori_Letter, "_", X_str, ".mat");
% % %     m = load(fname);
% % % 
% % %     % Find top 10 q's to test
% % %     s = struct();
% % %     NUM = 10;
% % %     [sortedA, indices] = sort(m.dist_from_edge_arr, 'descend');
% % %     q_to_try = zeros(NUM, 9);
% % %     for j=1:NUM
% % %         q_list = m.q_list_arr{indices(j)};
% % %         q = q_list(5,:);
% % %         q_to_try(j, :) = q;
% % %         key = strcat(ori_Letter,"_",sprintf('%02d',j));
% % %         mapObj(key) = q;
% % % 
% % %         disp("***")
% % %         disp(key)
% % %         disp(m.dist_from_edge_arr(indices(j)))
% % %         disp(q);
% % % 
% % %     end
% % % end
% % % 
% % % key_list = mapObj.keys;
% % % parfor key_num = 1:numel(mapObj.keys)
% % %     key = key_list{key_num};
% % %     q = mapObj(key);
% % % 
% % %     start = q;
% % %     goal = getfield(params, "q_home");
% % %     all_trajectory = planJointToJoint(panda_ec, panda_sc, env, start, goal, params);
% % % 
% % %     paths_struct = struct(key+"_to_home", all_trajectory, "home_to_"+key, flip(all_trajectory,1));
% % %     assert(motionCheck(panda_ec, panda_sc, env, paths_struct, params))
% % %     
% % %     SAVE_DIR = "./test_paths/";
% % %     writeCSV_from_paths_struct(paths_struct, SAVE_DIR, params);
% % % 
% % % end
% % % 
% % % 
% % % % plotCSV(panda_sc, "/home/oconnorlabmatlab/Code/libfranka/MATLAB/trajectory_planning/staging_area/test_paths/home_to_ori_A_01_10%.csv", env, params)
% % % % 
% % % % for A_num = 1:numel(stagingPositions)
% % % % 
% % % % 
% % % %         A_name = stagingPositions(A_num,:);
% % % % 
% % % %         % Skip different letters
% % % %         A_name_char = char(A_name);
% % % %         A_letter = A_name_char(end-1);
% % % % 
% % % %         % Plan paths
% % % %         start = getfield(params, A_name);
% % % %         goal = getfield(params, "q_home");
% % % %         all_trajectory = planJointToJoint(panda_ec, panda_sc, env, start, goal, params);
% % % %         
% % % %         % Check motion is OK
% % % %         paths_struct = struct(A_name+"_to_home", all_trajectory, "home_to_"+A_name, flip(all_trajectory,1));
% % % %         assert(motionCheck(panda_ec, panda_sc, env, paths_struct, params))
% % % % 
% % % %         % Write CSV
% % % %         writeCSV_from_paths_struct(paths_struct, SAVE_DIR, params);
% % % % 
% % % % %         show(panda_ec, params.stagingA0)
% % % % %         plotJointMotion(panda_sc, all_trajectory, env, params)
% % % % end
% % % % 
% % % % 
% % % % % Generate home_to_ori and ori_to_home trajectories
% % % % 
% % % % % Plot robot
% % % % indices = [1,5];
% % % % for i = 1:numel(indices)
% % % %     idx = indices(i);
% % % %     ori_num = 1;
% % % %     q_arr = q_best{ori_num};
% % % %     q = q_arr(idx, :);
% % % %     getTransform(panda_sc, q, "panda_hand_tcp")
% % % %     show(panda_sc, q); hold on;
% % % % end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Choose X = -0.65 based on edge_dist_cell
X = -0.65;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% Create poses
msg_array = [];
for ori_num =1:numel(ori_cell)
    
    % Read file
    ori_Letter = ori_names{ori_num};
    X_str = num2str(X);
    fname = strcat("./temp/", ori_Letter, "_", X_str, ".mat");
    m = load(fname);

    % Get q
    idx = find(m.dist_from_edge_arr==max(m.dist_from_edge_arr), 1);
    dist_from_edge = m.dist_from_edge_arr(idx);
    q_list_best = m.q_list_arr{idx};

%     % Get pose
%     ori = ori_cell{ori_num};
%     XYZ = [X;Y;Z];
%     T0 = construct_pose(ori, XYZ, OFFSET);
% 
%     % Calculate q that gives acceptable cartesian path
%     [dist_from_edge_best, q_list_best] = find_best_q_for_cartesian_path(T0, TRAVEL_DIST, panda_sc,ik, params);
%     assert(dist_from_edge_best>0.05);
%         
    % Adjust q's 7th joint for different rotations
    q_staging = q_list_best(end,:);
    j7 = q_staging(7);
    jValsStaging = j7:pi/2:j7+3*pi/2;
    jValsStaging(jValsStaging > params.jointMax(7)) = jValsStaging(jValsStaging >params.jointMax(7)) - 2*pi;      % roll jValsStaging to be within joint limits

    % Adjust q_extreme's 7th joint for diferent rotations;
    q_extreme = q_list_best(1,:);  % Most extreme position (monkey should be prevented from pulling further)
    j7 = q_extreme(7);
    jValsExtreme = j7:pi/2:j7+3*pi/2;
    jValsExtreme(jValsExtreme > params.jointMax(7)) = jValsExtreme(jValsExtreme > params.jointMax(7)) - 2*pi;

    for rot_num = 0:3

        % Update q_staging
        q_staging_update = q_staging;
        q_staging_update(7) = jValsStaging(rot_num+1);
        q_staging_update(8:9) = 0.01;  

        % Pose Name
        letters = 'ABCDE';
        poseLetter = letters(ori_num);
        poseName = "staging_" + poseLetter + num2str(rot_num);

        % Print result
        msg = poseName + " = [" + num2str(q_staging_update)+ "]";
        disp(msg)
        msg_array = [msg_array;msg];

        % Update q_extreme
        q_extreme_update = q_extreme;
        q_extreme_update(7) = jValsStaging(rot_num+1);
        q_extreme_update(8:9) = 0.01;  

        % Pose Name
        letters = 'ABCDE';
        poseLetter = letters(ori_num);
        poseName = "extreme_" + poseLetter + num2str(rot_num);

        % Print result
        msg = poseName + " = [" + num2str(q_extreme_update)+ "]";
        disp(msg)
        msg_array = [msg_array;msg];
    end
end


% Display msg
for row_num = 1:size(msg_array,1)
    msg = msg_array(row_num,:);
    disp(msg)
end

% Evaluate msg
for row_num = 1:size(msg_array,1)
    msg = msg_array(row_num,:);
    eval(msg)
end




%% Plot robot
plotJointMotion(panda_sc, q, env, params)
plot3(params.shelf_pts(:,1),params.shelf_pts(:,2),params.shelf_pts(:,3), "r*")






