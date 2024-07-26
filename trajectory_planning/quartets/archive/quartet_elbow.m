%% Generate elbow look-up-table for robot execution

clear; close all;
addpath("../..")
params = CustomParameters();
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment();
prefix = '20240722_';
SAVE_DIR = 'trajectories/elbow/';

ik = inverseKinematics('RigidBodyTree', panda_sc);
ik.SolverParameters.MaxIterations = 1000;


mkdir(SAVE_DIR);
addpath(SAVE_DIR);

%% Generate list of staging positions 
qA= load(['trajectories/',prefix,'qA.mat'], "qA");
qA = qA.qA;
qW = load(['trajectories/',prefix,'qW.mat'], "qW");
qW = qW.qW;

%% Construct LUT

z_shift = 0.2;
spacing = 0.0001;


i = 3;
arr = qA.qA_arr;

q_staging = arr(3,:);
T0 = getTransform(panda_sc, q_staging, 'panda_hand_tcp');
z_steps = T0(3,4):spacing:T0(3,4)+z_shift;

num_steps = numel(z_steps);

q_elbow_arr = zeros(num_steps,9);
q_elbow_arr(1,:) = q_staging;

for s_num = 2:num_steps
    
    Ts = T0;
    Ts(3,4) = z_steps(s_num);
    initial_guess = q_elbow_arr(s_num-1,:);
    
    [qs,solnInfo] = ik('panda_hand_tcp',Ts,[1 1 1 1 1 1],initial_guess);
    assert(~strcmp(solnInfo.Status, "best available"), "ik failed.")
    q_elbow_arr(s_num,:) = qs;

end


elbow_table = [(0:num_steps-1)',z_steps', q_elbow_arr(:,3), sign(q_elbow_arr(:,4))];

assert(elbow_table(1,2) < elbow_table(end,2), 'Z value not rising');
assert(elbow_table(1,3) == q_staging(3))
assert(elbow_table(end,3) == q_elbow_arr(end,3));
assert(max(abs(elbow_table(2:end,3)-elbow_table(1:end-1,3)))<0.001, "Large jump in elbow angle.")



