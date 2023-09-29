clear; close all;
addpath("..")
params = CustomParameters();
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();

%% Joint positions
% Exact
e_q = [1.37969,1.58302,-0.607506,-1.30402,-2.43018,2.12204,1.12848];
e_q(end+1:end+2) = 0.01; % Append values for grippers

% Calculated - read from csv
filename = "/home/oconnorlabmatlab/Code/libfranka/MATLAB/planned_paths/slots/01B00_above_to_slot_5%.csv";
anglesArray = readmatrix(filename);
fingerPositions = 0.01*ones([size(anglesArray,1),2]);
anglesArray = [anglesArray, fingerPositions];
c_q = anglesArray(end,:);

%% Poses
e_T = getTransform(panda_sc, e_q, "panda_hand_tcp")
c_T = getTransform(panda_sc, c_q, "panda_hand_tcp")
T = reshape([0.001088,0.0515128,-0.998662,0,-0.624884,0.779703,0.0395378,0,0.780712,0.624017,0.0330385,0,0.518029,0.549216,-0.104045,1],[4,4])


% Above position
% {"O_T_EE": [0.00157337,0.0491855,-0.998779,0,-0.625286,0.779487,0.0374013,0,0.78039,0.624475,0.031982,0,0.518612,0.548305,-0.0843891,1], "O_T_EE_d": [0.00108799,0.0515128,-0.998662,0,-0.624884,0.779703,0.0395378,0,0.780712,0.624017,0.0330385,0,0.518034,0.549211,-0.083845,1], "F_T_NE": [0.7071,-0.7071,0,0,0.7071,0.7071,0,0,0,0,1,0,0,0,0.1034,1], "NE_T_EE": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], "F_T_EE": [0.7071,-0.7071,0,0,0.7071,0.7071,0,0,0,0,1,0,0,0,0.1034,1], "EE_T_K": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], "m_ee": 0.73, "F_x_Cee": [-0.01,0,0.03], "I_ee": [0.001,0,0,0,0.0025,0,0,0,0.0017], "m_load": 0, "F_x_Cload": [0,0,0], "I_load": [0,0,0,0,0,0,0,0,0], "m_total": 0.73, "F_x_Ctotal": [-0.01,0,0.03], "I_total": [0.001,0,0,0,0.0025,0,0,0,0.0017], "elbow": [0.530308,-1], "elbow_d": [0.531415,-1], "elbow_c": [0,0], "delbow_c": [0,0], "ddelbow_c": [0,0], "tau_J": [0.747017,-44.5629,14.7824,9.86205,0.221989,-2.74873,0.214436], "tau_J_d": [0,0,0,0,0,0,0], "dtau_J": [20.5206,-25.1512,50.7555,-50.3282,47.8431,-6.02351,-41.7526], "q": [0.361567,1.47881,0.530308,-1.4219,2.88587,1.92019,0.375797], "dq": [0.0022126,0.00421691,-0.000746999,0.0023217,0.000133703,-8.23525e-05,0.000773539], "q_d": [0.362753,1.47877,0.531415,-1.42198,2.8873,1.91871,0.376333], "dq_d": [1.67311e-10,1.53211e-10,2.05391e-11,6.20393e-10,4.17444e-11,4.02123e-10,6.27276e-11], "ddq_d": [-1.05138e-07,-9.63674e-08,-1.29896e-08,-3.89688e-07,-2.62013e-08,-2.52687e-07,-3.94129e-08], "joint_contact": [0,0,0,0,0,0,0], "cartesian_contact": [0,0,0,0,0,0], "joint_collision": [0,0,0,0,0,0,0], "cartesian_collision": [0,0,0,0,0,0], "tau_ext_hat_filtered": [0.816477,-0.340449,0.722854,-0.210065,0.770215,-1.15152,0.268102], "O_F_ext_hat_K": [-3.93636,-0.70702,1.63072,1.28528,-1.05263,0.464896], "K_F_ext_hat_K": [-1.66973,1.97126,-3.46126,1.29935,-0.751739,-0.0272753], "O_dP_EE_d": [0,0,0,0,0,0], "O_ddP_O": [0,0,-9.81], "O_T_EE_c": [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], "O_dP_EE_c": [0,0,0,0,0,0], "O_ddP_EE_c": [0,0,0,0,0,0], "theta": [0.36162,1.47568,0.531345,-1.4212,2.88589,1.91988,0.375821], "dtheta": [0,0,0,0,0,0,0], "current_errors": [], "last_motion_errors": [], "control_command_success_rate": 0.97, "robot_mode": "Move", "time": 2991295, "CPU_steady_clock_time_ms": 1916327984.800150}
a_q = [0.361567,1.47881,0.530308,-1.4219,2.88587,1.92019,0.375797, 0.01, 0.01]
a_T = reshape([0.00157337,0.0491855,-0.998779,0,-0.625286,0.779487,0.0374013,0,0.78039,0.624475,0.031982,0,0.518612,0.548305,-0.0843891,1], [4,4]);

% Above calculated
b_q = anglesArray(1,:);
b_T = getTransform(panda_sc, a_q, "panda_hand_tcp")

%% Obervations
% Calculated and exact q both give the same pose --> so MATLAB's model is
% good
% This calculated pose aligns with the measured pose from the real robot.
%
% potenial issues:
% - Is the robot simply lagging behind due to low impedance? Incresaing
% impedance helped accuracy, however, robot appears to be where it is being
% commanded. Next idea is to enforce that the pose aligns with the +Z axis
% to ensure that pulling up is going straight up.
% - Is the robot inaccurate