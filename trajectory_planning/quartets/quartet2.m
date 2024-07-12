%% Load variables
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Set random stream for reproducibility
seedval = 123;
stream = RandStream('mt19937ar', 'Seed', seedval);
RandStream.setGlobalStream(stream);

%% Load robot with quartert
[panda_ec_orig, panda_sc_orig] = loadPandaWithShape(params);
[panda_ec_A, panda_sc_A] = loadPandaWithShape(params);
[panda_ec_W, panda_sc_W] = loadPandaWithShape(params);
env = build_collision_environment;

% Reduce joint limits
JOINT_REDUCTION = 0.3;
J7_REDUCTION = 0.2;
W_SHIFT = [0, 0, -params.cylinderRadius];


% Joints 1-6
for body_num = 1:6
    oldLimits = panda_sc_orig.Bodies{body_num}.Joint.PositionLimits;
    newLimits = oldLimits + [JOINT_REDUCTION,-JOINT_REDUCTION];
    panda_sc_A.Bodies{body_num}.Joint.PositionLimits = newLimits;
    panda_sc_W.Bodies{body_num}.Joint.PositionLimits = newLimits;

end

% Joint 7
body_num = 7;
oldLimits = panda_sc_orig.Bodies{body_num}.Joint.PositionLimits;
newLimits = oldLimits + [J7_REDUCTION,-J7_REDUCTION];
panda_sc_A.Bodies{body_num}.Joint.PositionLimits = newLimits;
panda_sc_W.Bodies{body_num}.Joint.PositionLimits = newLimits;

% Restrict joint 1 to be positive
body_num = 1;
oldLimits = panda_sc_W.Bodies{body_num}.Joint.PositionLimits;
newLimits = [0, oldLimits(2)];
panda_sc_A.Bodies{body_num}.Joint.PositionLimits = newLimits;
panda_sc_W.Bodies{body_num}.Joint.PositionLimits = newLimits;


% Restrict joint 2 to be positive for panda_A
body_num = 2;
oldLimits = panda_sc_A.Bodies{body_num}.Joint.PositionLimits;
newLimits = [0, oldLimits(2)];
panda_sc_A.Bodies{body_num}.Joint.PositionLimits = newLimits;


% Restrict joint 2 to be negative for panda_W
body_num = 2;
oldLimits = panda_sc_W.Bodies{body_num}.Joint.PositionLimits;
newLimits = [oldLimits(1), 0];
panda_sc_W.Bodies{body_num}.Joint.PositionLimits = newLimits;

%% Set up inverse kinematics
ik_A = inverseKinematics('RigidBodyTree',panda_sc_A);
ik_A.SolverParameters.MaxIterations = 1000;
ik_W = inverseKinematics('RigidBodyTree',panda_sc_W);
ik_W.SolverParameters.MaxIterations = 1000;
ik_orig = inverseKinematics('RigidBodyTree', panda_sc_orig);
ik_orig.SolverParameters.MaxIterations = 1000;

theta_list = -pi/4:pi/2:5*pi/4;

% ss = ManipulatorStateSpaceSphere(panda_ec, panda_sc);
% sv = ManipulatorStateValidatorSphere(ss, env, params.validationDistance,params.radius_offset, params);
% sv.IgnoreSelfCollision = false;
% sv.Environment = env;





%% Find optimal XYZ position
XYZ_list = [];
for X = -0.65:-0.02:-0.80
    for Y = 0.0:0.02:0.20
        for Z = (12.5:1:25.5)*0.0254
            XYZ = [X,Y,Z];
            XYZ_list = [XYZ_list;XYZ];
        end
    end
end

% Get list of quartet shape names
body_names = {};
for i = panda_sc_orig.NumBodies-3:panda_sc_orig.NumBodies
    body_names{end+1} = panda_sc_orig.BodyNames{i};
end

% 
% %% Explore positions for horizontal
% 
% parfor XYZ_num = 1:size(XYZ_list,1)
% 
% %     disp(strcat(num2str(XYZ_num), " of ", num2str(size(XYZ_list,1))))
% 
%     warning('off', 'all');
% 
%     XYZ = XYZ_list(XYZ_num,:);
% 
%     qA_success = zeros(numel(body_names), numel(theta_list));
%     qW_success = zeros(numel(body_names), numel(theta_list));
%     for body_num = 1:numel(body_names)
%         body_name = body_names{body_num};
%         for theta_num = 1:numel(theta_list)
%             theta = theta_list(theta_num);
% 
%             % Calculate correct TA and TW
%             [TA, TW] = calc_TA_TW(XYZ, theta, W_SHIFT);
% 
%             % Vertical
%             initialGuess = randomConfiguration(panda_sc_A);
%             [qA, solnInfoA] = find_q_from_T(panda_sc_A,body_name, TA, ik_A, initialGuess);
% 
%             % Horizontal
%             initialGuess = randomConfiguration(panda_sc_W);
%             [qW, solnInfoW] = find_q_from_T(panda_sc_W,body_name,TW, ik_W, initialGuess);
% 
% 
% 
% 
%             % Record
%             if (~is_robot_in_self_collision_ignore_pairs(panda_sc_A, qA)) && (strcmp(solnInfoA.Status, "success"))
%                 qA_success(body_num, theta_num) = 1;
%             end
% 
%             % Record
%             if (~is_robot_in_self_collision_ignore_pairs(panda_sc_W, qW)) && (strcmp(solnInfoW.Status, "success"))
%                 qW_success(body_num, theta_num) = 1;
%             end
% 
%         end
%     end
% 
%     Anum_invalid = numel(qA_success)-sum(sum(qA_success));
%     Wnum_invalid = numel(qW_success)-sum(sum(qW_success));
% 
%     if (Wnum_invalid ==0) &&  (Anum_invalid == 0)
%         result = strcat("W: ", num2str(XYZ), "      : ", num2str(Wnum_invalid),"\n","A: ", num2str(XYZ), "      : ", num2str(Anum_invalid));
%         disp(result)
% %         result = strcat("A: ", num2str(XYZ), "      : ", num2str(Anum_invalid));
% %         disp(result)
% %         disp("*****")
%     end
%     
% end


%% Select XYZ from the above results
XYZ = [-0.67, 0.06, 0.6477];

% Perform IK
qA_arr = [];
qW_arr = [];

for body_num = 1:numel(body_names)
    body_name = body_names{body_num};
    for theta_num = 1:numel(theta_list)
        theta = theta_list(theta_num);
    
        % Calculate correct TA and TW
        [TA, TW] = calc_TA_TW(XYZ, theta, W_SHIFT);

        % Do IK for 100 valid q's
        num_loops = 100;
        qA_loop = [];
        qW_loop = [];
        for loop = 1:num_loops
            guessA = randomConfiguration(panda_sc_A);
            guessA(8:9)=0.01;
            guessW = randomConfiguration(panda_sc_W);
            guessW(8:9)=0.01;
            [qA, solnInfoA] = find_q_from_T(panda_sc_A,body_name, TA, ik_A, guessA);
            [qW, solnInfoW] = find_q_from_T(panda_sc_W,body_name, TW, ik_W, guessW);
    
            if strcmp(solnInfoA.Status, "success")
                qA_loop = [qA_loop;qA];
            end
            if strcmp(solnInfoW.Status, "success")
                qW_loop = [qW_loop;qW];
            end

        end

        % Choose two q's that are furthest apart on limb 1
        [submin, minidx] = min(qA_loop(:,1));
        [submax, maxidx] = max(qA_loop(:,1));
        qAa = qA_loop(minidx,:);
        qAb = qA_loop(maxidx,:);
% 
        [submin, minidx] = min(qW_loop(:,1));
        [submax, maxidx] = max(qW_loop(:,1));
        qWa = qW_loop(minidx,:);
        qWb = qW_loop(maxidx,:);
   

        qA_arr = [qA_arr;qAa;qAb];
        qW_arr = [qW_arr;qWa;qWb];

    end
end

% % Sanity check:
% plotJointMotion(panda_sc_A, qA_arr(17,:), env, params); hold on;
% show(panda_sc_A, qA_arr(18,:))
% 
% plotJointMotion(panda_sc_W, qW_arr(17,:), env, params); hold on;
% show(panda_sc_W, qW_arr(18,:))


arr = qW_arr;

inter = zeros(size(arr));
inter_shift = zeros(4);
inter_shift(1,4) = 0.05;  % Positive x direction, towards base

num_positions = size(arr,1);

%% Calculate inter positions
for i = 1:num_positions
    
    q = arr(i,:);
    T = getTransform(panda_sc_orig, q, 'panda_hand_tcp');
    T_inter = T + inter_shift;
    [q_inter, solnInfo] = ik_orig('panda_hand_tcp', T_inter, [1 1 1 1 1 1], q);
    
    % Checks
    assert(strcmp(solnInfo.Status,"success"))
    assert(~is_robot_in_self_collision_ignore_pairs(panda_sc_orig, q_inter))
    collisions = checkCollision(panda_sc_orig, q_inter, env);
    env_collision = collisions(2);
    assert(~env_collision)
    assert(sum(abs(q_inter-q))<1.0)
    
    inter(i,:) = q_inter;
end


% Calculate staging to inter
cell_staging_to_inter = {};
for i = 1:num_positions

    start = arr(i,:);
    goal = inter(i,:);

    [traj, planned_path] = planJointToJoint(panda_ec_orig, panda_sc_orig, env, start, goal, params);
    cell_staging_to_inter{end+1} = {traj, planned_path};

    disp(size(traj))
    disp(size(planned_path))
end
save("cell_staging_to_inter.mat","cell_staging_to_inter")


% Calculating inter to inter
cell_inter_to_inter = cell(num_positions, num_positions);

parfor r=1:num_positions
    for c = 1:num_positions
    if c<=r
        continue
    end

    disp([r,c])
    start = inter(r,:);
    goal = inter(c,:);
    [traj, planned_path] = planJointToJoint(panda_ec_orig, panda_sc_orig, env, start, goal, params);
%     plotJointMotion(panda_sc_orig, traj, env, params)
    cell_inter_to_inter{r,c} = {traj, planned_path};

    
    end
end
save("cell_inter_to_inter.mat","cell_inter_to_inter")

traj_lengths = [];
for i = 1:numel(cell_inter_to_inter)

    subcell = cell_inter_to_inter{i};

    if numel(subcell) == 0
        continue
    end

    traj = subcell{1};
    planned_path = subcell{2};

    traj_length = size(traj,1);
    traj_lengths = [traj_lengths;traj_length];

%     if traj_length > 5000
%         break
%     end

end

hist(traj_lengths, 50)
plotJointMotion(panda_sc_W, traj, env, params)
% Why are some paths taking 6 seconds



%% Identify any random configurations that work with all inter
rand_configs = [];
sv = construct_state_validator(panda_ec_orig, panda_sc_orig, env, params)
count = 0;
while true
    count = count+1;
    if mod(count, 100)==0
        disp(count)
    end
    q_random = randomConfiguration(panda_sc_orig);
    q_random(8:9) = 0.01;

    % Test if in self collision
    if is_robot_in_self_collision_ignore_pairs(panda_sc_orig, q_random)
        continue
    end

    % Test if in environmental collision
    if ~sv.isStateValid(q_random)
        continue
    end

    % Test if direct path possible
    start = q_random;
    all_valid = true;
    for i = 1:num_positions
        goal = inter(i,:);
        [planned_path, isDirectValid] = check_direct_path(sv, start, goal);
        if ~isDirectValid
            all_valid = false;
            break
        end
    end

    if all_valid
        rand_configs = [rand_configs; q_random]
    end

end




% traj_lengths_diff = [];
% traj_lengths_same = [];
% for r=1:num_positions
%     for c = 1:num_positions
%     if c<=r 
%         continue
%     end
%     traj_and_path = cell_inter_to_inter{r,c};
%     traj = traj_and_path{1};
% 
%     traj_length = size(traj,1);
% 
%     if traj_length >4
% 
%     traj_lengths = [traj_lengths;size(traj,1)];
%     end
% end
% 
% hist(traj_lengths, 50)







% 
% 
% 
% %% Plot Joint to Joint given intermediate step
% qA_inter = zeros(size(qA_arr));
% qW_inter = zeros(size(qW_arr));
% 
% arr = qA_arr;
% inter = zeros(size(arr));
% 
% num_staging = size(arr,1);
% 
% for i = 1:size(arr,1)
%     
%     q = arr(i,:);
%     T = getTransform(panda_sc_orig, q, 'panda_hand_tcp');
%     T_inter = T;
%     T_inter(3,4) = T_inter(3,4) - 0.025;
%     [q_inter, solnInfo] = ik_orig('panda_hand_tcp', T_inter, [1 1 1 1 1 1], q);
%     
%     % Checks
%     assert(strcmp(solnInfo.Status,"success"))
%     assert(~is_robot_in_self_collision_ignore_pairs(panda_sc_orig, q_inter))
%     collisions = checkCollision(panda_sc_orig, q_inter, env);
%     env_collision = collisions(2);
%     assert(~env_collision)
%     assert(sum(abs(q_inter-q))<0.5)
%     
%     inter(i,:) = q_inter;
% end
% 
% %% Plan paths - staging to inter
% 
% cell_staging_to_inter = {};
% for i = 1:num_staging
% 
%     start = arr(i,:);
%     goal = inter(i,:);
% 
%     [traj, planned_path] = planJointToJoint(panda_ec_orig, panda_sc_orig, env, start, goal, params);
% %     plotJointMotion(panda_sc_orig, traj, env, params)
%     cell_staging_to_inter{end+1} = {traj, planned_path};
% 
%     disp(size(traj))
%     disp(size(planned_path))
% end
% 
% %% Plan paths - inter to inter
% cell_inter_to_inter = cell(num_staging, num_staging);
% 
% parfor r=1:num_staging
%     for c = 1:num_staging
%     if c<=r
%         continue
%     end
% 
%     disp([r,c])
%     start = inter(r,:);
%     goal = inter(c,:);
%     [traj, planned_path] = planJointToJoint(panda_ec_orig, panda_sc_orig, env, start, goal, params);
% %     plotJointMotion(panda_sc_orig, traj, env, params)
%     cell_inter_to_inter{r,c} = {traj, planned_path};
% 
%     end
% end
% 
% 
% 
% for r=1:num_staging
%     for c = 1:num_staging
% 
%         if c<=r
%             continue
%         end
%         subcell = cell_inter_to_inter{r,c};
%         traj = subcell{1};
%         planned_path = subcell{2};
% %         disp(size(traj,1))
%         disp(size(planned_path,1))
%     end
% end
% 
% %% Plan paths - staging to staging (no inter)
% cell_staging_to_staging = cell(num_staging, num_staging);
% 
% parfor r=1:num_staging
%     for c = 1:num_staging
%     if c<=r
%         continue
%     end
% 
%     disp([r,c])
%     start = arr(r,:);
%     goal = arr(c,:);
%     [traj, planned_path] = planJointToJoint(panda_ec_orig, panda_sc_orig, env, start, goal, params);
% %     plotJointMotion(panda_sc_orig, traj, env, params)
%     cell_staging_to_staging{r,c} = {traj, planned_path};
% 
%     end
% end
% 
% 
% 
% for r=1:num_staging
%     for c = 1:num_staging
% 
%         if c<=r
%             continue
%         end
%         subcell = cell_staging_to_staging{r,c};
%         traj = subcell{1};
%         planned_path = subcell{2};
%         disp(size(traj,1))
% %         disp(size(planned_path,1))
%     end
% end
% 
% plotJointMotion(panda_sc_orig, traj, env, params)