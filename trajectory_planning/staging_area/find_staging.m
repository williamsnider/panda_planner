% Code to find the best configurations for the staging area / monkey grasp

clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

%% Inputs
shape_offset = 0.07;

%% Construct list of orientation
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
% 
% % Rotations about x-axis 90deg
% th1 = pi/2;
% R1 = [1 0 0; 0 cos(th1) -sin(th1);  0 sin(th1) cos(th1)];
% 
% % Ori B - pointing down, peg towards hallway
% ori_B = R1 * ori_A;
% 
% % Ori C - pointing down, peg toward ceiling
% ori_C = R1 * ori_B;
% 
% % Ori D - pointing up, peg AWAY from hallway
% ori_D = R1 * ori_C;
% 
% % Rotations about z-axis 90 deg
% th2 = -pi/2;
% R2 = [cos(th2) -sin(th2) 0; sin(th2) cos(th2) 0; 0 0 1];
% 
% % ori E - pointing toward monkey, peg down
% ori_E = R2 * ori_A;
% 
% % Ori W - pointing down/right, peg down/towards hallway
% th3 = pi/4;
% R3 = [1 0 0;    0 cos(th3) -sin(th3);  0 sin(th3) cos(th3)];
% th4 = -pi/4;
% R4 = [cos(th4) -sin(th4) 0; sin(th4) cos(th4)  0; 0  0 1];
% ori_W = R3 * R4 * ori_A;
% 
% % Ori_X - pointing down/left, peg up/towards hallway
% ori_X = R1 * ori_W;
% 
% % ori_Y - pointing up/left, peg up/away from hallway
% ori_Y = R1 *ori_X;
% 
% % ori_Z - pointing up/right, peg down/awaway from hallway
% ori_Z = R1 * ori_Y;
% 
% % ori_cell = {ori_A,ori_B, ori_C, ori_D, ori_E, ori_W, ori_X, ori_Y, ori_Z};
% % ori_names = {"ori_A","ori_B", "ori_C", "ori_D", "ori_E","ori_W", "ori_X", "ori_Y", "ori_Z"};
% ori_cell = {ori_C, ori_D, ori_E};
% ori_names = {"ori_C", "ori_D", "ori_E"};


%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();

% Reduce joint limits by 0.3 rad in
reduction = 0.5;

for body_num = 1:7
    oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
    newLimits = oldLimits + [reduction,-reduction];
    panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;
end


ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
weights = [1 1 1 1 1 1];



% Construct Transformation matrices
% XYZ = [-0.75; 0.0; 17*0.0254];
% T_cell = {};
% for cell_num = 1:numel(ori_cell)
%     T_cell{cell_num} = construct_pose(ori_cell{cell_num}, XYZ, shape_offset);
% end
%
% % Plot T_cell
% for cell_num = 1:numel(T_cell)
%     plotTransforms(se3(T_cell{cell_num}), 'FrameSize',0.05); hold on;
% end

XYZ_arr = [];
for X = -0.80:0.025:-0.725
    for Y = 0.0:0.025:0.2
        for Z = (10:22)*0.0254
            XYZ_arr = [XYZ_arr;X,Y,Z];
        end
    end
end

% Find valid q's using ik
XYZ_valid = [];
parfor XYZ_num = 1:size(XYZ_arr,1)
%     disp(num2str(XYZ_num) + " of " +num2str(size(XYZ_arr,1)))

    XYZ = XYZ_arr(XYZ_num,:);

    T_cell = {};
    for cell_num = 1:numel(ori_cell)
        T_cell{cell_num} = construct_pose(ori_cell{cell_num}, XYZ', shape_offset);
    end

    q_arr = zeros(numel(T_cell),9);
    for cell_num = 1:numel(T_cell)
        T = T_cell{cell_num};
        initialGuess = randomConfiguration(panda_sc);
        initialGuess(8:9) = 0.01;
        [q_initial,solnInfo] = ik('panda_hand_tcp',T,[1 1 1 1 1 1],initialGuess);

        if strcmp(solnInfo.Status, "best available")
            q_initial = zeros(1,9);
            break
        end

        q_arr(cell_num,:) = q_initial;

    end

    if any(any(q_arr==0))
        continue
    else
%         XYZ_valid = [XYZ_valid;XYZ];
%         disp("*******")
        disp(XYZ)
%         disp(q_arr)
    end


end

XYZ = [-0.75, 0.175, 0.5588];
T_cell = {};
for cell_num = 1:numel(ori_cell)
    T_cell{cell_num} = construct_pose(ori_cell{cell_num}, XYZ', shape_offset);
end

T_q_extreme 

q_arr = zeros(numel(T_cell),9);
for cell_num = 1:numel(T_cell)
    T = T_cell{cell_num};
    initialGuess = randomConfiguration(panda_sc);
    initialGuess(8:9) = 0.01;
    [q_initial,solnInfo] = ik('panda_hand_tcp',T,[1 1 1 1 1 1],initialGuess);

    if strcmp(solnInfo.Status, "best available")
        q_initial = zeros(1,9);
    end

    q_arr(cell_num,:) = q_initial;
end
disp(q_arr)

for q_num = 1:size(q_arr, 1)

    if q_num <4
        continue
    end

    show(panda_sc, q_arr(q_num,:)); hold on;
end
