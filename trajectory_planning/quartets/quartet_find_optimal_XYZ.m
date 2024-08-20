function quartet_find_optimal_XYZ()
%% QUARTET_FIND_OPTIMAL_XYZ Identifies the XYZ position that works for presenting the quartet in 4 rotations in both the A and W orientations
% We need to identify a position in the robot's workspace that allows it to
% present each stimuli of in the quartet at 4 orientations each, for both
% the A  (pointing toward monkey) orientation. This
% function does this using robot models with slightly reduced joint limits,
% ensuring that there is still some range of motion available at the
% identified XYZ. This function will display the XYZ positions that work
% for A and W. Choose the one you want and run it as an input with the
% quartet_find_optimal_q_extreme script.

%% Set RNG for reproducibilitiy
seedval = 123;
stream = RandStream('mt19937ar', 'Seed', seedval);
RandStream.setGlobalStream(stream);

%% Clear all variables and loam parameters
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

%% Load quartet-common variables
[panda_ec_orig, panda_sc_orig, panda_ec_A, panda_sc_A, ik_orig, ik_A, env, body_names, theta_list, X_SHIFT_EXTREME_TO_STAGING] = load_quartet_common_variables(params);

%% Create list of XYZ positions to test
XYZ_list = [];
for X = -0.65:-0.02:-0.75
    for Y = 0.06:0.02:0.10
        for Z = (19.5:1:25.5)*0.0254
            XYZ = [X,Y,Z];
            XYZ_list = [XYZ_list;XYZ];
        end
    end
end

%% Test whether each XYZ position is valid for both A and W orientations
SV = construct_state_validator(panda_ec_orig, panda_sc_orig, env, params);
panda_sc_restricted = panda_sc_A;
ik_restricted = ik_A;

% XYZ_list = [-0.67, 0.08, 0.6477];

parfor XYZ_num = 1:size(XYZ_list,1)
    warning('off', 'all');

    XYZ = XYZ_list(XYZ_num,:);

    qA_success = zeros(numel(body_names), numel(theta_list));
%     qW_success = zeros(numel(body_names), numel(theta_list));

    any_invalid = false;
    for body_num = 1:numel(body_names)
        body_name = body_names{body_num};

        if any_invalid==true
            break
        end
        
        for theta_num = 1:numel(theta_list)
            
            if any_invalid==true
                break
            end
            
            theta = theta_list(theta_num);

            % Calculate correct T
            T_extreme = calc_TA(XYZ, theta);
            T_extreme_to_staging = zeros(4);
            T_extreme_to_staging(1,4) = X_SHIFT_EXTREME_TO_STAGING;
            T_staging_to_inter = zeros(4);
            T_staging_to_inter(1,4) = 0.05;

            early_stop_if_any_found = true;
            [q_extreme_12, q_staging_12, q_inter_12, elbow_12] = find_q_extreme_staging_inter(SV, T_extreme, body_name, T_extreme_to_staging, T_staging_to_inter, panda_sc_restricted, panda_sc_orig, ik_restricted, ik_orig, early_stop_if_any_found, env, params);

            
            if size(q_extreme_12,1)==2
                qA_success(body_num, theta_num) = 1;
            else
                any_invalid=true;
            end
            %             % Horizontal - extreme
%             initialGuess = randomConfiguration(panda_sc_A);
%             [qA_extreme, solnInfoA_extreme] = find_q_from_T(panda_sc_A,body_name, TA, ik_A, initialGuess);
% 
%             % Horizontal - staging
%             T_staging = TA;
%             T_staging(1,4) = T_staging(1,4) + X_SHIFT_EXTREME_TO_STAGING;
%             initialGuess = randomConfiguration(panda_sc_A);
%             [qA_staging, solnInfoA_staging] = find_q_from_T(panda_sc_orig,body_name, T_staging, ik_A, qA_extreme);
% % 
% 
%             dist = sum(abs(qA_staging-qA_extreme));
% 
% %             % Vertical
% %             initialGuess = randomConfiguration(panda_sc_W);
% %             [qW, solnInfoW] = find_q_from_T(panda_sc_W,body_name,TW, ik_W, initialGuess);
% 
%             % Record
%             if (~is_robot_in_self_collision_ignore_pairs(panda_sc_A, qA_extreme)) && (strcmp(solnInfoA_extreme.Status, "success") && ~is_robot_in_self_collision_ignore_pairs(panda_sc_orig, qA_staging)) && (strcmp(solnInfoA_staging.Status, "success") && dist<2.0)
%                 qA_success(body_num, theta_num) = 1;
%             end
% 
% %             % Record
% %             if (~is_robot_in_self_collision_ignore_pairs(panda_sc_W, qW)) && (strcmp(solnInfoW.Status, "success"))
% %                 qW_success(body_num, theta_num) = 1;
% %             end

        end
    end

    Anum_invalid = numel(qA_success)-sum(sum(qA_success));
%     Wnum_invalid = numel(qW_success)-sum(sum(qW_success));

%     if (Anum_invalid ==0) &&  (Wnum_invalid == 0)
%         result = strcat("W: ", num2str(XYZ), "      : ", num2str(Wnum_invalid),"\n","A: ", num2str(XYZ), "      : ", num2str(Anum_invalid));
%         disp(result)
%     end

    if (Anum_invalid ==0)
        result = strcat("A: ", num2str(XYZ), "      : ", num2str(Anum_invalid));
        disp(result)
    end
end

end
