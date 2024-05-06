% Code to find the best configurations different assembly types


%% Load variables
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Set random stream for reproducibility
stream = RandStream('mt19937ar', 'Seed', 123); 
RandStream.setGlobalStream(stream);

%% Inputs
JOINT_REDUCTION = 0.2;
J7_REDUCTION = 0.2;

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();

% Reduce range of motion of joints for safety
% Restrict joints 1-6 by JOINT_REDUCTION
for body_num = 1:6
    oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
    newLimits = oldLimits + [JOINT_REDUCTION,-JOINT_REDUCTION];
    panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;
end

% Restrict joint 7 by J7_REDUCTION
body_num = 7;
oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
newLimits = oldLimits + [J7_REDUCTION,-J7_REDUCTION];
panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;

% Remove custom shape
removeBody(panda_sc, 'panda_custom_shape')

%% Create 5-shape linear array
% Define the properties of the new cylinders
cylinderLength = 0.05;  % Length of the cylinder
cylinderRadius = 0.025;  % Radius of the cylinder
cylinderSpacing = 0.075;
cylinderLengthOffset = 0.07;  % Dist from TCP along long axis to center of grasping
TCP_Z_OFFSET = 0.0;
TCP_LENGTH_OFFSET = 0.11;

numPoints = 20;  % Number of points around the cylinder

% Create cylinder mesh data
[xc, yc, zc] = cylinder(cylinderRadius, numPoints);
zc(2, :) = cylinderLength;  % Adjust cylinder length

% Convert cylindrical coordinates to vertices and faces
vertices = [xc(:), yc(:), zc(:)];
faces = convhull(xc(:), yc(:), zc(:));

% Shift cylinder so that joint position is inside the cylidner
vertices(:,3) = vertices(:,3) - cylinderLength;

triObj = triangulation(faces, vertices);

% Save the mesh as an STL file
stlFileName = 'cylinderMesh.stl';
stlwrite(triObj, stlFileName);

% Get the transformation matrix of 'panda_hand_tcp'
tcpTransform = getTransform(panda_sc, params.q_home, 'panda_hand_tcp');

% Start creating and attaching cylinders
body_names = {};
for i = 1:5
    % Calculate the offset in the x-direction for each cylinder
%     yOffset = (i - 3) * cylinderSpacing;  % Center cylinder at 0 offset
    th_list = -pi/2:pi/4:pi/2 + pi;
    theta = th_list(i);
    Rx = [1 0 0;
      0 cos(theta) -sin(theta);
      0 sin(theta) cos(theta)];

    % Define the joint that connects the cylinder to the hand
    joint = rigidBodyJoint(['joint_', num2str(i)], 'fixed');
    T_cylinder = eye(4);
    T_cylinder(3,4) = T_cylinder(3,4) + TCP_LENGTH_OFFSET;
    Tx = eye(4);
    Tx(1:3,1:3) = Rx;
    T_cylinder = Tx * T_cylinder;
    T_cylinder(3,4) = T_cylinder(3,4) + TCP_Z_OFFSET;
    setFixedTransform(joint, T_cylinder);  % Offset position

    % Create the cylinder body
    body_name = ['body_', num2str(i)];
    body_names{end+1} = body_name;
    body = rigidBody(['body_', num2str(i)]);
    body.Joint = joint;

    % Add visual mesh to the body
    addVisual(body, 'Mesh', stlFileName);

    % Attach to the robot tree
    addBody(panda_sc, body, 'panda_hand_tcp');
end



%% Do inverse kinematics for shape positions

XYZ = [-0.60, 0.2, 20*0.0254];
initialGuess = randomConfiguration(panda_sc);

for body_num = 1:numel(body_names)
    body_name = body_names{body_num};
    for theta = -pi/4:pi/2:5*pi/4
        ori_base = eye(3);
        ori_A0 = ori_base;
        
        T_A0 = eye(4);
        Rz = [cos(theta) -sin(theta) 0; 
              sin(theta) cos(theta) 0; 
              0 0 1];
        T_A0(1:3,1:3) = Rz;
        T_A0(1:3,4) = XYZ';
        
        T = T_A0;
        ik = inverseKinematics('RigidBodyTree',panda_sc);
        ik.SolverParameters.MaxIterations = 1000;
        initialGuess(8:9) = 0.01;
        [q,solnInfo] = ik(body_name,T,[1 1 1 1 1 1],initialGuess);
        
        % Check that ik was successful
        if ~strcmp(solnInfo.Status, "success")
            disp("IK failed")
        end
        
        % Display distance between positions
        dist = sum(abs(initialGuess-q));
        disp(dist);

        initialGuess = q;
        
        %% Show result
        plot_assembly(panda_sc, q, triObj, cylinderRadius, ori_base, XYZ)
        
        disp(strcat(body_name, "    ",num2str(theta)));
        input("");
    end
end


