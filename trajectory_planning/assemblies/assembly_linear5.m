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
JOINT_REDUCTION = 0.4;
J7_REDUCTION = 0.3;

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

% Restrict joint 1 to be positive
body_num = 1;
oldLimits = panda_sc.Bodies{body_num}.Joint.PositionLimits;
newLimits = [0, oldLimits(2)];
panda_sc.Bodies{body_num}.Joint.PositionLimits = newLimits;



% Remove custom shape
removeBody(panda_sc, 'panda_custom_shape')


%% Create 5-shape linear array
% Define the properties of the new cylinders
cylinderLength = 0.05;  % Length of the cylinder
cylinderRadius = 0.025;  % Radius of the cylinder
cylinderSpacing = 0.075;

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
    yOffset = (i - 3) * cylinderSpacing;  % Center cylinder at 0 offset

    % Define the joint that connects the cylinder to the hand
    joint = rigidBodyJoint(['joint_', num2str(i)], 'fixed');
    setFixedTransform(joint, trvec2tform([0, yOffset, params.hand_to_tcp + cylinderLength]));  % Offset position

    % Create the cylinder body
    body_name = ['body_', num2str(i)];
    body_names{end+1} = body_name;
    body = rigidBody(['body_', num2str(i)]);
    body.Joint = joint;

    % Add visual mesh to the body
    addVisual(body, 'Mesh', stlFileName);

    % Add collision mesh to the body
    collisionBody = collisionCylinder(cylinderRadius, 2*cylinderLength);
    addCollision(body, collisionBody);

    % Attach to the robot tree
    addBody(panda_sc, body, 'panda_hand');
end



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
parfor XYZ_num = 1:size(XYZ_list,1)
    warning('off', 'all');

    XYZ = XYZ_list(XYZ_num,:);

    q_success = zeros(numel(body_names), numel(theta_list));
    for body_num = 1:numel(body_names)
        body_name = body_names{body_num};
        for theta_num = 1:numel(theta_list)
            theta = theta_list(theta_num);

            [q, solnInfo] = find_XYZ_q(panda_sc,body_name, XYZ, theta, ik);       


            % Check that ik was successful
            if ~strcmp(solnInfo.Status, "success")
                %                         disp("IK failed")
            end

            % Record
            if (~checkCollision(panda_sc, q)) && (strcmp(solnInfo.Status, "success"))
                q_success(body_num, theta_num) = 1;
            end



            %% Show result
            %         plot_assembly(panda_sc, q, triObj, cylinderRadius, ori_base, XYZ)
        end
    end

    num_invalid = numel(q_success)-sum(sum(q_success));
    if num_invalid==0
        result = strcat(num2str(XYZ), "      : ", num2str(num_invalid));
        disp(result)
    end
end


%% Chose XYZ
XYZ = [-0.60, 0.1, 0.5207];

for body_num = 1:numel(body_names)
    body_name = body_names{body_num};
    for theta_num = 1:numel(theta_list)
        theta = theta_list(theta_num);

        [q, solnInfo] = find_XYZ_q(panda_sc,body_name, XYZ, theta, ik);       


        % Check that ik was successful
        if ~strcmp(solnInfo.Status, "success")
            disp("IK failed")
        end

        % Check that not in collision
        if checkCollision(panda_sc, q)
            disp("Self collision")
        end




        %% Show result
        plot_assembly(panda_sc, q, triObj, cylinderRadius, XYZ)
        input("")
    end
end

