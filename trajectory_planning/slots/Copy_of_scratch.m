clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Load variables
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment;
ik = inverseKinematics('RigidBodyTree', panda_sc);
ik.SolverParameters.MaxIterations = 1000;

%
q_orig=[ -1.77237, 0.542475, -0.0491045, -1.20432, -2.32845, 2.99899, 0.0147864, 0.1, 0.1];

T_orig = getTransform(panda_sc, q_orig, "panda_hand_tcp");

% Rotate T such that xyz is the same but 3rd column of T_orig (direction
% robot is pointing), is in xy plane (no z component). Keep 2nd column
% identical (i.e. rotate about this). Call this T_new.

% Perform inverse kinematics, identify 10 q_new's that achieve this T_new
R_orig = T_orig(1:3, 1:3);
z_dir = R_orig(:,3);

% Project z_dir onto the XY-plane and normalize
z_xy = [z_dir(1:2); 0];
z_xy = z_xy / norm(z_xy);

% Keep the y-axis (2nd column) unchanged
y_axis = R_orig(:,2);

% Compute new x-axis using cross product: x = y × z
x_axis = cross(y_axis, z_xy);
x_axis = x_axis / norm(x_axis);

% Recompute z-axis to ensure orthogonality
z_axis = cross(x_axis, y_axis);

R_new = [x_axis, y_axis, z_axis];
T_new = T_orig;
T_new(1:3,1:3) = R_new;
 


weights = [1 1 1 1 1 1];
initialGuess = q_orig;
q_new = zeros(10, 9);

for i = 1:10
    [q_sol, solnInfo] = ik('panda_hand_tcp', T_new, weights, randomConfiguration(panda_sc));
    disp(solnInfo.Status)
    q_new(i,:) = q_sol;
end

show(panda_sc, q_orig); hold on;
show(panda_sc, q_new(2,:));
format short
disp(q_new(7,:))
