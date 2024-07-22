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

% Plot panda
plotJointMotion(panda_sc_orig, randomConfiguration(panda_sc_orig), env, params);
hold on;
xyz = params.shelf_pts;
plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'r*')

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
% plotJointMotion(panda_sc_A, qA_arr(1,:), env, params); hold on;
% show(panda_sc_A, qA_arr(1,:))
% show(panda_sc_W, qW_arr(1,:))

%% Inputs for qA vs qW

% Position names
staging_letters = ["W","X","Y","Z"];

% q array to use
varName = 'qW';
eval(['q_arr = ',varName,'_arr;'])
inter_shift = zeros(4);
inter_shift(1,4) = 0.05;  % Positive x direction, towards base


% % Position names
% % staging_letters = 
% staging_letters = ["A","B","C","D"]; % ["W","X","Y","Z"];
% 
% % q array to use
% varName = 'qA';
% eval(['q_arr = ',varName,'_arr;'])
% inter_shift = zeros(4);
% inter_shift(1,4) = 0.05;  % Positive x direction, towards base
% inter_shift(3,4) = -0.05; % Down, towards floor

% Where to save
savedir = strcat(params.CustomParametersDir,'/trajectory_planning/quartets/trajectories/');
mkdir(savedir)
prefix = "20240722";

% Seed rng for substituting short paths for reproducibility

%% Aggregate names of positions


%% Calculate staging_to_inter and inter_to_inter paths/trajectories
[cell_staging_to_inter_70, cell_staging_to_inter_path, cell_inter_to_inter_70, cell_inter_to_inter_path] = calc_between_staging_paths(q_arr,inter_shift,panda_ec_orig, panda_sc_orig, ik_orig, env, params);




%% Substitute short paths - find seedval that has similar distributions of same and different motions
seedval_sub_short_paths = 123; % qW
target_min = 2250;
target_max = 4000;
% seedval_sub_short_paths = 127; % qA
% target_min = 2000;
% target_max = 4000;
[cell_inter_to_inter_sub_70,cell_inter_to_inter_sub_path] = calc_substituted_paths(seedval_sub_short_paths, cell_inter_to_inter_70,cell_inter_to_inter_path, target_min, target_max);

plot_histogram_of_traj_lengths(cell_inter_to_inter_sub_70);
waitforbuttonpress();

%% Convert to full path - piece together staging_to_inter, inter_to_inter, and inter_to_staging
num_positions = size(cell_inter_to_inter_path, 1);
cell_full_70 = cell(num_positions, num_positions);
cell_full_path = cell(num_positions, num_positions);
for r=1:num_positions
    for c = 1:num_positions

        if r==c
            continue
        end

        % Combine
        full_traj = [cell_staging_to_inter_70{r}; cell_inter_to_inter_sub_70{r,c}; flip(cell_staging_to_inter_70{c},1)];
        cell_full_70{r,c} = full_traj;

        full_path = [cell_staging_to_inter_path{r}; cell_inter_to_inter_sub_path{r,c}; flip(cell_staging_to_inter_path{c},1)];
        cell_full_path{r,c} = full_path;

    end
end





%% Check full trajectories
sv = construct_state_validator(panda_ec_orig, panda_sc_orig, env, params);
for r = 1:num_positions
    for c=1:num_positions
        if r==c
            continue
        end

        disp([r,c])
        % Load traj
        traj = cell_full_70{r,c};

        % Check kinematics and start/stop
        assert(checkTrajKinematics(traj,q_arr(r,:), q_arr(c,:), params))

        % Check no self collisions
        assert(~checkTrajForSelfCollisions(panda_sc_orig, traj, params));
    end
end

%% Calculate trajectories for 10% and 40% speed
cell_full_10 = cell(size(cell_full_70));
cell_full_40 = cell(size(cell_full_70));
for r = 1:num_positions
    for c=1:num_positions
        if c<=r
            continue
        end

        disp([r,c])
        % Load planned path
        planned_path = cell_full_path{r,c};

        % Calculate trajectories - 10%
        params_copy = params;
        params_copy.vScale = 0.1;
        params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
        traj = joint_path_to_traj(planned_path, params_copy);
        assert(checkTrajKinematics(traj, planned_path(1,:), planned_path(end,:), params_copy)); % Check
        assert(~checkTrajForSelfCollisions(panda_sc_orig, traj, params));
        traj_reverse = flip(traj,1);
        assert(checkTrajKinematics(traj_reverse, planned_path(end,:), planned_path(1,:), params_copy)); % Check
        cell_full_10{r,c} = traj;
        cell_full_10{c,r} = traj_reverse;

        % Calculate trajectories - 40%
        params_copy = params;
        params_copy.vScale = 0.4;
        params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
        traj = joint_path_to_traj(planned_path, params_copy);
        assert(checkTrajKinematics(traj, planned_path(1,:), planned_path(end,:), params_copy)); % Check
        assert(~checkTrajForSelfCollisions(panda_sc_orig, traj, params));
        traj_reverse = flip(traj,1);
        assert(checkTrajKinematics(traj_reverse, planned_path(end,:), planned_path(1,:), params_copy)); % Check
        cell_full_40{r,c} = traj;
        cell_full_40{c,r} = traj_reverse;
    end
end

%% Save mat files

% Dynamically create struct, populate
eval([varName ' = struct();']);
eval([varName, '.cell_staging_to_inter_70 = cell_staging_to_inter_70'])
eval([varName, '.cell_staging_to_inter_path = cell_staging_to_inter_path'])
eval([varName, '.cell_inter_to_inter_70 = cell_inter_to_inter_70'])
eval([varName, '.cell_inter_to_inter_path = cell_inter_to_inter_path'])
eval([varName, '.cell_full_10 = cell_full_10'])
eval([varName, '.cell_full_40 = cell_full_40'])
eval([varName, '.cell_full_70 = cell_full_70'])
eval([varName, '.cell_full_path = cell_full_path'])
eval([varName, '.', varName, '_arr = ', varName, '_arr'])

% Save the struct to a single .mat file
save(strcat(savedir, prefix,"_",varName,".mat"), varName);

%% Save Trajectories
mkdir(savedir);
addpath(savedir);
mkdir(strcat(savedir, varName));
addpath(strcat(savedir, varName));


for r = 1:num_positions
    for c = 1:num_positions

        if r==c
            continue
        end

        disp([r,c])

        speed_factor = 10;
        traj = cell_full_10{r,c};
        traj7 = traj(:,1:7);
        n1 = pos_names{r};
        n2 = pos_names{c};
        fname = strcat(savedir,varName,"/",prefix, "_staging", n1,"_to_", "staging",n2,"_",num2str(speed_factor),"%.csv");
        writematrix(traj7,fname)

        speed_factor = 40;
        traj = cell_full_40{r,c};
        traj7 = traj(:,1:7);
        n1 = pos_names{r};
        n2 = pos_names{c};
        fname = strcat(savedir,varName,"/",prefix, "_staging", n1,"_to_", "staging",n2,"_",num2str(speed_factor),"%.csv");
        writematrix(traj7,fname)


        speed_factor = 70;
        traj = cell_full_70{r,c};
        traj7 = traj(:,1:7);
        n1 = pos_names{r};
        n2 = pos_names{c};
        fname = strcat(savedir,varName,"/",prefix, "_staging", n1,"_to_", "staging",n2,"_",num2str(speed_factor),"%.csv");
        writematrix(traj7,fname)

    end
end


