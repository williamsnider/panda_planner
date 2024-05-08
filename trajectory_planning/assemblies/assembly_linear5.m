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

q = randomConfiguration(panda_sc);
q(8:9) = 0.01;


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
traj_arr = {};
for row = 2:size(q_arr,1)
    disp(row)
    start = q_arr(row-1,:);
    goal = q_arr(row,:);
    
    % Use original joint limits
    [traj, planned_path] = planJointToJoint(panda_ec, panda_sc, env, start, goal, params);
    traj_arr{end+1} = traj;


end
save("traj_arr","traj_arr")


%% Plot joint motion in steps
for i = 1:numel(traj_arr)
    traj = traj_arr{i};
    plotJointMotion(panda_sc, traj, env, params)
    input("")
end



all_traj = [];

for i= 1:numel(traj_arr)
    all_traj = [all_traj;traj_arr{i}];
end
save("all_traj","all_traj")
plotJointMotion(panda_sc, all_traj, env, params)