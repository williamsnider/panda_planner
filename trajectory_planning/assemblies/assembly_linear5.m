% Code to find the best configurations different assembly types


%% Load variables
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Set random stream for reproducibility
stream = RandStream('mt19937ar', 'Seed', 123);
RandStream.setGlobalStream(stream);

%% Load common assembly parameters
c = assembly_common("linear", 4, params);

body_names = c.body_names;
panda_sc = c.panda_sc;
panda_ec = c.panda_ec;
env = c.env;

%% Do inverse kinematics for shape positions
XYZ_list = [];
for X = -0.60:-0.02:-0.70
    for Y = 0.10:0.02:0.20
        for Z = (12.5:1:22.5)*0.0254
            XYZ = [X,Y,Z];
            XYZ_list = [XYZ_list;XYZ];
        end
    end
end

theta_list = -pi/4:pi/2:5*pi/4;


ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
% parfor XYZ_num = 1:size(XYZ_list,1)
%     warning('off', 'all');
%
%     XYZ = XYZ_list(XYZ_num,:);
%
%     q_success = zeros(numel(body_names), numel(theta_list));
%     for body_num = 1:numel(body_names)
%         body_name = body_names{body_num};
%         for theta_num = 1:numel(theta_list)
%             theta = theta_list(theta_num);
%
%             [q, solnInfo] = find_XYZ_q(panda_sc,body_name, XYZ, theta, ik);
%
%
%             % Check that ik was successful
%             if ~strcmp(solnInfo.Status, "success")
%                 %                         disp("IK failed")
%             end
%
%             % Record
%             if (~checkCollision(panda_sc, q)) && (strcmp(solnInfo.Status, "success"))
%                 q_success(body_num, theta_num) = 1;
%             end
%
%
%
%             %% Show result
%             %         plot_assembly(panda_sc, q, triObj, cylinderRadius, ori_base, XYZ)
%         end
%     end
%
%     num_invalid = numel(q_success)-sum(sum(q_success));
%     if num_invalid==0
%         result = strcat(num2str(XYZ), "      : ", num2str(num_invalid));
%         disp(result)
%     end
% end


%% Chose XYZ
% XYZ = [-0.62, 0.18, 0.5207];
XYZ = [-0.62, 0.10, 0.5207];

q = [1.798459128682102   1.552800000000000   1.402202106961650  -2.118002688471622   1.497955826674127   1.417562303241438   0.791853618601684   0.010000000000000   0.010000000000000];


q_arr = [];
for body_num = 1:numel(body_names)
    body_name = body_names{body_num};
    for theta_num = 1:numel(theta_list)
        theta = theta_list(theta_num);

        initialGuess = q; % Use last as seed


        attempt_num = 1;
        while attempt_num < 10
            [q, solnInfo] = find_XYZ_q(panda_sc,body_name, XYZ, theta, ik, initialGuess);
            if (strcmp(solnInfo.Status, "success") && ~checkCollision(panda_sc, q))
                break
            end

            % Try again with new seed
            initialGuess = randomConfiguration(panda_sc);
            initialGuess(8:9) = 0.01;
            attempt_num = attempt_num + 1;
        end

        % Check that ik was successful
        if ~strcmp(solnInfo.Status, "success")
            disp("IK failed")
        end

        % Check that not in collision
        if checkCollision(panda_sc, q)
            disp("Self collision")
        end

        q_arr = [q_arr;q];




%         %% Show result
%         plot_assembly(panda_sc, q, triObj, cylinderRadius, XYZ)
%         input("")
    end
end

%% Remove joint limit constraints
for body_num = 1:7
    panda_sc.Bodies{body_num}.Joint.PositionLimits = c.panda_sc_orig.Bodies{body_num}.Joint.PositionLimits;
    panda_ec.Bodies{body_num}.Joint.PositionLimits = c.panda_ec_orig.Bodies{body_num}.Joint.PositionLimits;
end


%% Simulate joint motion
num_bodies = numel(body_names);
num_traj = num_bodies * 4 - 1;
traj_arr_10 = cell(num_traj,1);
traj_arr_40 = cell(num_traj,1);
traj_arr_70 = cell(num_traj,1);
path_arr = cell(num_traj,1);

parfor row = 2:size(q_arr,1)
    disp(row)
    start = q_arr(row-1,:);
    goal = q_arr(row,:);
    params_copy = params;

    % 10% max speed
    params_copy.vScale = 0.1;
    params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;

    [traj, planned_path] = planJointToJoint(panda_ec, panda_sc, env, start, goal, params_copy);
    path_arr{row-1} = planned_path;
    traj_arr_10{row-1} = traj;
    
    % 40% max speed
    params_copy.vScale = 0.4;
    params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
    traj = joint_path_to_traj(planned_path, params_copy);
    assert(checkTrajectory(traj, start, goal, params_copy)); % Check
    traj_arr_40{row-1} = traj;

    % 70% max speed
    params_copy.vScale = 0.7;
    params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
    traj = joint_path_to_traj(planned_path, params_copy);
    assert(checkTrajectory(traj, start, goal, params_copy)); % Check
    traj_arr_70{row-1} = traj;

end

save("traj_arr_10","traj_arr_10")
save("traj_arr_40","traj_arr_40")
save("traj_arr_70","traj_arr_70")
save("path_arr", "path_arr")

%% Save trajectories
letters = ["A","B","C","D","E"];




%% Plot joint motion in steps
arr_cell = traj_arr_70;
for i = 1:numel(arr_cell)
    traj = arr_cell{i};
    plotJointMotion(panda_sc, traj, env, params)
    input("")
end

%% Compare lengths
for i = 1:num_traj
    result = strcat(num2str(i),": ", num2str(size(traj_arr_10{i},1)), " ",num2str(size(traj_arr_40{i},1)), " ",num2str(size(traj_arr_70{i},1)), " ");
    disp(result)
end


%% home to staging
home_to_staging_traj_arr_10 = cell(num_bodies*4,1);
home_to_staging_traj_arr_40 = cell(num_bodies*4,1);
home_to_staging_traj_arr_70 = cell(num_bodies*4,1);
home_to_staging_path_arr = cell(num_bodies*4,1);

parfor row = 1:size(q_arr,1)
    disp(row)
    params_copy = params;

    start = params_copy.q_home;
    goal = q_arr(row,:);
    params_copy = params;

    % 10% max speed
    params_copy.vScale = 0.1;
    params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;

    [traj, planned_path] = planJointToJoint(panda_ec, panda_sc, env, start, goal, params_copy);
    home_to_staging_path_arr{row} = planned_path;
    home_to_staging_traj_arr_10{row} = traj;
    
    % 40% max speed
    params_copy.vScale = 0.4;
    params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
    traj = joint_path_to_traj(planned_path, params_copy);
    assert(checkTrajectory(traj, start, goal, params_copy)); % Check
    home_to_staging_traj_arr_40{row} = traj;

    % 70% max speed
    params_copy.vScale = 0.7;
    params_copy.vMaxAll = params_copy.vMaxAllAbsolute*params_copy.vScale;
    traj = joint_path_to_traj(planned_path, params_copy);
    assert(checkTrajectory(traj, start, goal, params_copy)); % Check
    home_to_staging_traj_arr_70{row} = traj;

end


save("home_to_staging_traj_arr_10","home_to_staging_traj_arr_10")
save("home_to_staging_traj_arr_40","home_to_staging_traj_arr_40")
save("home_to_staging_traj_arr_70","home_to_staging_traj_arr_70")
save("home_to_staging_path_arr", "home_to_staging_path_arr")

arr_cell = home_to_staging_traj_arr_70;
for i = 1:numel(arr_cell)
    traj = arr_cell{i};
    plotJointMotion(panda_sc, traj, env, params)
    input("")
end

%% Generate name_list
letters = ["A","B","C","D"];
name_list = [];
for i=1:numel(letters)
    letter = letters(i);
    for j = 0:3
        name_list = [name_list, strcat(letter,num2str(j))];
    end
end


%% Write CSV's
SAVE_DIR = "trajectories/";
PREFIX = "20240509";

speed_list = ["10","40","70"];
for speed_num = 1:numel(speed_list)
    speed_factor = speed_list(speed_num);
    arr_name = strcat("home_to_staging_traj_arr_",speed_factor);
    arr_name = char(arr_name);
    arr = eval(arr_name);
    for i=1:numel(arr)
        
        % home to staging
        fname = strcat(SAVE_DIR,PREFIX,"_home_to_staging", name_list(i),"_",speed_factor,"%.csv")
        traj = arr{i};
        writematrix(traj(:, 1:7), fname)
    
        % staging to home
        traj=flip(traj,1);
        fname = strcat(SAVE_DIR,PREFIX,"_staging", name_list(i),"_to_home_",speed_factor,"%.csv")
        writematrix(traj(:, 1:7), fname)
    
    end
end

% 
% for n2 = 2:numel(name_list)
%     n1 = n2-1;


speed_list = ["10","40","70"];
for speed_num = 1:numel(speed_list)
    speed_factor = speed_list(speed_num);
    arr_name = strcat("traj_arr_",speed_factor);
    arr_name = char(arr_name);
    arr = eval(arr_name);
    for i=1:numel(arr)
        
        name1 = name_list(i);
        name2 = name_list(i+1);

        % name1 to name2
        fname = strcat(SAVE_DIR,PREFIX,"_staging",name1,"_to_staging",name2,"_",speed_factor,"%.csv")
        traj = arr{i};
        writematrix(traj(:, 1:7), fname)
    
        % name2 to name1
        traj=flip(traj,1);
        fname = strcat(SAVE_DIR,PREFIX,"_staging",name2,"_to_staging",name1,"_",speed_factor,"%.csv")
        writematrix(traj(:, 1:7), fname)
    
    end
end
% end


% Plot forward
for n2=2:numel(name_list)
    n1 = n2-1;

    name1 = name_list(n1);
    name2 = name_list(n2);
    
    fname = strcat(SAVE_DIR,PREFIX,"_staging",name1,"_to_staging",name2,"_",speed_factor,"%.csv");
    plotCSV(panda_sc, fname, env, params);
    input("")
end




all_traj = [];

for i= 1:numel(traj_arr)
    all_traj = [all_traj;traj_arr{i}];
end
save("all_traj","all_traj")
plotJointMotion(panda_sc, all_traj, env, params)