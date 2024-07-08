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
env = build_collision_environment;

% Reduce joint limits
JOINT_REDUCTION = 0.3;
J7_REDUCTION = 0.2;

% Duplicate the robot models
panda_ec = panda_ec_orig;
panda_sc = panda_sc_orig;

% Joints 1-6
for body_num = 1:6
    oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
    newLimits = oldLimits + [JOINT_REDUCTION,-JOINT_REDUCTION];
    panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;
end

% Joint 7
body_num = 7;
oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
newLimits = oldLimits + [J7_REDUCTION,-J7_REDUCTION];
panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;

% % Restrict joint 1 to be positive
% body_num = 1;
% oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
% newLimits = [0, oldLimits(2)];
% panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;
%
% % Restrict joint 2 to be positive
% body_num = 2;
% oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
% newLimits = [0, oldLimits(2)];
% panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;

%% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
theta_list = -pi/4:pi/2:5*pi/4;

ss = ManipulatorStateSpaceSphere(panda_ec, panda_sc);
sv = ManipulatorStateValidatorSphere(ss, env, params.validationDistance,params.radius_offset, params);
sv.IgnoreSelfCollision = false;
sv.Environment = env;





%% Find optimal XYZ position
XYZ_list = [];
for X = -0.65:-0.02:-0.80
    for Y = 0.0:0.02:0.20
        for Z = (12.5:1:22.5)*0.0254
            XYZ = [X,Y,Z];
            XYZ_list = [XYZ_list;XYZ];
        end
    end
end

% Get list of quartet shape names
body_names = {};
for i = panda_sc.NumBodies-3:panda_sc.NumBodies
    body_names{end+1} = panda_sc.BodyNames{i};
end


%% Explore positions for horizontal


% %% Scratch section
% 
% theta = theta_list(4);
% XYZ = [-0.75, 0.1, 20*0.0254];
% W_SHIFT = [0.045, 0, 0.03];
% body_name = 'panda_cylinder2';
% 
% % Position pointing vertically
% T_A0 = eye(4);
% Rz = [cos(theta) -sin(theta) 0;
%     sin(theta) cos(theta) 0;
%     0 0 1];
% T_A0(1:3,1:3) = Rz;
% T_A0(1:3,4) = XYZ';
% TA = T_A0;
% 
% % Position pointing horizontally
% T_W0 = eye(4);
% Ry = [cos(-pi/2) 0 sin(-pi/2);
%        0 1 0;
%       -sin(-pi/2) 0 cos(-pi/2)];
% T_W0(1:3,1:3) = Ry*Rz;
% T_W0(1:3,4) = XYZ';
% TW = T_W0;
% 
% % Do IK
% initialGuess = randomConfiguration(panda_sc);
% [qA,solnInfoA] = ik(body_name,TA,[1 1 1 1 1 1],initialGuess);
% show(panda_sc, qA); hold on;
% 
% initialGuess = randomConfiguration(panda_sc);
% [qW,solnInfoW] = ik(body_name,TW,[1 1 1 1 1 1],initialGuess);
% 
% % Plot transforms
% % plotTransforms(se3(TA)); hold on;
% % plotTransforms(se3(TW));
% 
% show(panda_sc, qW);







W_SHIFT = [0.045, 0, 0.03];
parfor XYZ_num = 1:size(XYZ_list,1)

%     disp(strcat(num2str(XYZ_num), " of ", num2str(size(XYZ_list,1))))

    warning('off', 'all');

    XYZ = XYZ_list(XYZ_num,:);

    qA_success = zeros(numel(body_names), numel(theta_list));
    qW_success = zeros(numel(body_names), numel(theta_list));
    for body_num = 1:numel(body_names)
        body_name = body_names{body_num};
        for theta_num = 1:numel(theta_list)
            theta = theta_list(theta_num);

            % Calculate correct TA and TW
            [TA, TW] = calc_TA_TW(XYZ, theta, W_SHIFT)

            % Vertical
            initialGuess = randomConfiguration(panda_sc);
            [qA, solnInfoA] = find_q_from_T(panda_sc,body_name, TA, ik, initialGuess);

            % Horizontal
            initialGuess = randomConfiguration(panda_sc);
            [qW, solnInfoW] = find_q_from_T(panda_sc,body_name,TW, ik, initialGuess);




            % Record
            if (~checkCollision(panda_sc, qA)) && (strcmp(solnInfoA.Status, "success"))
                qA_success(body_num, theta_num) = 1;
            end

            % Record
            if (~checkCollision(panda_sc, qW)) && (strcmp(solnInfoW.Status, "success"))
                qW_success(body_num, theta_num) = 1;
            end

        end
    end

    Anum_invalid = numel(qA_success)-sum(sum(qA_success));
    Wnum_invalid = numel(qW_success)-sum(sum(qW_success));

    if (Wnum_invalid ==0) &&  (Anum_invalid == 0)
        result = strcat("W: ", num2str(XYZ), "      : ", num2str(Wnum_invalid),"\n","A: ", num2str(XYZ), "      : ", num2str(Anum_invalid));
        disp(result)
%         result = strcat("A: ", num2str(XYZ), "      : ", num2str(Anum_invalid));
%         disp(result)
%         disp("*****")
    end
    
end


%% Select XYZ from the above results
XYZ = [-0.65, 0.1, 0.5715];

% Perform IK
qA_arr = [];
qW_arr = [];
guessA = randomConfiguration(panda_sc);
guessA(8:9)=0.01;
guessW = randomConfiguration(panda_sc);
guessW(8:9)=0.01;
for body_num = 1:numel(body_names)
    body_name = body_names{body_num};
    for theta_num = 1:numel(theta_list)
        theta = theta_list(theta_num);
    
        % Calculate correct TA and TW
        [TA, TW] = calc_TA_TW(XYZ, theta, W_SHIFT);

        % Do IK
        [qA, solnInfoA] = find_q_from_T(panda_sc,body_name, TA, ik, guessA);
        [qW, solnInfoW] = find_q_from_T(panda_sc,body_name, TW, ik, guessW);

        assert(strcmp(solnInfoA.Status, "success"))
        assert(strcmp(solnInfoW.Status, "success"))

        qA_arr = [qA_arr;qA];
        qW_arr = [qW_arr;qW];

        guessA = qA;
        guessW = qW;

    end
end



