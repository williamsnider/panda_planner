clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Load variables
[panda_ec, panda_sc] = loadPandaWithShape(params);
env = build_collision_environment;
ik = inverseKinematics('RigidBodyTree', panda_sc);
ik.SolverParameters.MaxIterations = 1000;

quartet_fname = params.CustomParametersDir+"/trajectory_planning/quartets/trajectories/20241009_A.mat";
save_dir = params.CustomParametersDir+"/trajectory_planning/slots/close/";
quartet_slots_csv = params.CustomParametersDir+"/trajectory_planning/slots/quartet_slots.csv";
date_prefix = "20250418_";
slot_dir = "20241112_manual_slots";

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

slot_id_list = {'08C42', '06A52', '06C20', '09A30', '04C32', '05C32'};

i = 1;
slot_name = slot_id_list{i};

% Read T
slot_fname = strcat(pwd,"/",slot_dir, "/",slot_name+".txt");
[q_downIn, val] = readSlot(slot_fname);
T_downIn = reshape(val.O_T_EE,4,4);

% Test random q, see if it works with sequences needed
% initialGuess = q_downIn;
for j=1:5
initialGuess = randomConfiguration(panda_ec);
weights = [1 1 1 1 1 1];
[q_downIn_ik,solnInfo] = ik('panda_hand_tcp',T_downIn,weights,initialGuess);
T_downIn_ik = getTransform(panda_sc, q_downIn_ik, "panda_hand_tcp");

q_downIn
q_downIn_ik
solnInfo
end

q_slot = q_downIn  % TODO: Change this to ik 

ss = ManipulatorStateSpaceEllipsoid(panda_ec, panda_sc);
sv = ManipulatorStateValidatorEllipsoid(ss, env, params.validationDistance,params.ellipsoid_radius_offset, params);
sv.IgnoreSelfCollision = false;
sv.Environment = env;

T_upIn = T_downIn;
ABOVE_HEIGHT = params.ABOVE_HEIGHT;
T_upIn(3,4) = T_upIn(3,4) + ABOVE_HEIGHT;

% Calculate minimum OUT_DIST to pull shape within sphere
for OUT_DIST = 0.0:0.005:0.4

    % Calculate T_upOut
    T_upOut = T_upIn;
    T_upOut(1:3,4) = T_upOut(1:3,4) - T_upOut(1:3,3)*OUT_DIST;

%     % Calculate T_out
%     T_out = T_slot;
%     T_out(1:3,4) = T_out(1:3,4) - T_out(1:3,3)*OUT_DIST;

    % Do inverse kinematics
    initialGuess = q_slot;
    weights = [1 1 1 1 1 1];
    [q_upOut,solnInfo] = ik('panda_hand_tcp',T_upOut,weights,initialGuess);

    % Break loop if valid (inside ellipsoidf for both T_upOut and T_downOut)
    if sv.isStateValid(q_upOut)

        % Test if q_downOut valid too
        T_downOut = T_upOut;
        T_downOut(3,4)= T_downIn(3,4);
        [q_downOut,solnInfo] = ik('panda_hand_tcp',T_downOut,weights,q_upOut);
        if sv.isStateValid(q_downOut)
            break
        end
    end

    if  OUT_DIST == 0.2
%         plotJointMotion(panda_sc, q_upOut, env, params)
%         disp('here')
    end
end

assert(sv.isStateValid(q_upOut))



initialGuess = q_slot;
weights = [1 1 1 1 1 1];
[q_pivot,solnInfo] = ik('panda_hand_tcp',T_pivot,weights,initialGuess);
assert(strcmp(solnInfo.Status,'success'))

% Create direct motion path
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, q_downIn));
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, (q_downIn+q_pivot)/2));
assert(~is_robot_in_self_collision_ignore_pairs(panda_sc, q_pivot));
paths_struct = struct();
paths_struct.wpts_downIn_to_pivot = [q_downIn; q_pivot]; % Direct motion;
[traj_10, traj_10_reverse, traj_40, traj_40_reverse, traj_70, traj_70_reverse] = planned_path_to_traj_10_40_70(paths_struct.("wpts_downIn_to_pivot") , panda_sc,params);
paths_struct.("downIn_to_pivot_10") = traj_10;
paths_struct.("downIn_to_pivot_40") = traj_40;
paths_struct.("downIn_to_pivot_70") = traj_70;
paths_struct.("pivot_to_downIn_10") = traj_10_reverse;
paths_struct.("pivot_to_downIn_40") = traj_40_reverse;
paths_struct.("pivot_to_downIn_70") = traj_70_reverse;

combined = [paths_struct.downIn_to_pivot_10; paths_struct.pivot_to_downIn_10];
plotJointMotion(panda_sc, combined, env,params)
