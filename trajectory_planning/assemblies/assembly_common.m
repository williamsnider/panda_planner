function c = assembly_common(assembly_type, num_bodies, params)

%% Struct to store parameters
c = struct();

%% Inputs
JOINT_REDUCTION = 0.2;
J7_REDUCTION = 0.1;
cylinderLength = 0.05;  % Length of the cylinder
cylinderRadius = 0.025;  % Radius of the cylinder
cylinderSpacing = 0.075; % (linear array)
TCP_Z_OFFSET = 0.0;  % (radial array)
TCP_LENGTH_OFFSET = 0.11; % (radial array)

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



%% Create mesh of cylinder as shape
numPoints = 20;
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

%% Attach shapes to robot
body_names = {};

if strcmp(assembly_type, "linear")
    
    % Calculate the offset in the x-direction for each cylinder
    yOffsets = (1:num_bodies)*cylinderSpacing;
    yOffsets = yOffsets - (yOffsets(1) + yOffsets(end)) /2;
    
    for i = 1:num_bodies
        yOffset = yOffsets(i);  % Center cylinder at 0 offset
    
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
elseif  strcmp(assembly_type, "radial")
    
    % Calculate rotations
    radial_spacing = pi/4;
    th_list = (1:num_bodies)*radial_spacing;
    th_list = th_list - (th_list(1)+th_list(end))/2;
    for i=1:num_bodies
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
        T_cylinder(3,4) = T_cylinder(3,4) + TCP_Z_OFFSET + params.hand_to_tcp;
        setFixedTransform(joint, T_cylinder);  % Offset position
    
        % Create the cylinder body
        body_name = ['body_', num2str(i)];
        body_names{end+1} = body_name;
        body = rigidBody(['body_', num2str(i)]);
        body.Joint = joint;
    
        % Add visual mesh to the body
        addVisual(body, 'Mesh', stlFileName);
    
        % Add collision mesh to the body
        collisionBody = collisionCylinder(cylinderRadius, cylinderLength);
        addCollision(body, collisionBody);
    
        % Attach to the robot tree
        addBody(panda_sc, body, 'panda_hand');
    end
else
    error(strcat("Invalid assembly type ", assembly_type));
end

%% Assign to struct
c.body_names = body_names;
c.panda_sc = panda_sc;
c.panda_ec = panda_ec;
c.env = env;
c.triObj = triObj;
c.cylinderRadius = cylinderRadius;
end

