function [panda_ec, panda_sc] = loadPandaWithShape(params)
%LOADPANDAWITHSHAPE Loads the Franka Emika Panda with an attached shape
% 
% The robot is constructed from a urdf (as opposed to working with matlab's
% built-in model of the panda) because this exposes this collision files
% (e.g. link0.stl) which is utilized by ManipulatorRRTSphere to check if
% the manipuatlor is within a sphere (i.e. not colliding with the shelves).



    %% Generate points in case anything was updated
    run generate_collision_point_lists.m

    %% Import robots (self collision and environment collision version)
    urdf_sc = strcat(fileparts(mfilename('fullpath')),'/panda_description/panda_sc.urdf');
    panda_sc = importrobot(urdf_sc, 'DataFormat', 'row' );
    
    urdf_ec = strcat(fileparts(mfilename('fullpath')),'/panda_description/panda_ec.urdf');
    panda_ec = importrobot(urdf_ec, 'DataFormat', 'row' );
    
    urdf_p = strcat(fileparts(mfilename('fullpath')),'/panda_description/panda_ec.urdf');
    panda_p = importrobot(urdf_p);
%     %% Attach quartet as shape
%     cylinderLength = params.cylinderLength;
%     cylinderRadius = params.cylinderRadius;
%     cylinderSpacing = 0.075; % (linear array)
%     num_bodies = 4; % quartet
% 
%      % Calculate the offset in the x-direction for each cylinder
%     yOffsets = (1:num_bodies)*cylinderSpacing;
%     yOffsets = yOffsets - (yOffsets(1) + yOffsets(end)) /2;
%     
%     for i = 1:num_bodies
%         yOffset = yOffsets(i);  % Center cylinder at 0 offset
%     
%         % Define the joint that connects the cylinder to the hand
%         joint = rigidBodyJoint(['joint_', num2str(i)], 'fixed');
%         tcp_to_base_of_stimulus = 0.0261;  % Distance between the TCP and the flat portion of each stimulus.
%         setFixedTransform(joint, trvec2tform([0, yOffset, params.hand_to_tcp + tcp_to_base_of_stimulus]));  % Offset position
%     
%         % Create the cylinder body
%         body_name = ['panda_cylinder', num2str(i)];
% %         body_names{end+1} = body_name;
%         body = rigidBody(body_name);
%         body.Joint = joint;
%     
%         %Add visual mesh to the body
%         stlFileName = strcat(params.CustomParametersDir,'/trajectory_planning/robot/panda_description/cylinder',num2str(i),'.stl');
%         addVisual(body, 'Mesh', stlFileName);
%     
% 
% 
%         % Add collision mesh to the body
%         collisionBody = collisionCylinder(cylinderRadius, 1.5*cylinderLength);  % Since the cylinder is centered, need 2*cylinderLength
%         collisionBody.Pose(3,4) = collisionBody.Pose(3,4) + 0.5*cylinderLength/2; % full length extending, half length extending back
%         addCollision(body, collisionBody);
%     
%         % Attach to the robot tree
%         addBody(panda_sc, body, 'panda_hand');
%         addBody(panda_ec, body, 'panda_hand');
%     end

    %% Attach single cylinder as shape
%     cylinderRadius = params.cylinderRadius;
%     cylinderLength = params.cylinderLength;
% 
%     % Construct cylinder points
%     numPoints = 20;
%     th = linspace(0, 2*pi, numPoints+1);
%     xc = repmat(cylinderRadius*cos(th),[2,1]);
%     yc = repmat(cylinderRadius*sin(th),[2,1]);
%     zc = zeros(size(xc));
%     zc(2,:) = cylinderLength;
% 
%     % Convert cylindrical coordinates to vertices and faces
%     vertices = [xc(:), yc(:), zc(:)];
%     faces = convhull(xc(:), yc(:), zc(:));
%     
%     % Shift cylinder so that joint position is inside the cylidner
%     vertices(:,3) = vertices(:,3) - cylinderLength;
%    
%     triObj = triangulation(faces, vertices);
%     
%     % Save the mesh as an STL file
%     for i = 1:5
%     stlFileName = strcat(fileparts(mfilename('fullpath')),'/panda_description/cylinder',num2str(i),'.stl');
%     stlwrite(triObj, stlFileName);
%     end

%     disp("WARNING: Panda does not have a shape attached by default.")
%     %% Assign collision and visual meshes for shape
%     % prevents collisions between grasped shape and world
% 
%     % Create shape body/joint
%     collisionCylinderLength = 0.1;
%     collisionCylinderRadius = 0.025;
%     offset = 0.01;  % Prevent cylinder from being in collision with fingertips
%     cylinder = collisionCylinder(collisionCylinderRadius,collisionCylinderLength);
%     shapeBody = rigidBody("panda_custom_shape");
%     shapeJoint = rigidBodyJoint("shapeJoint");
%     setFixedTransform(shapeJoint,[0,0,0.1034+collisionCylinderLength/2 + offset,0], 'dh');      % Place shape into the end effector gripper.
% 
%     % Add collision mesh
%     T = eye(4);
%     addCollision(shapeBody,cylinder,T);
%     shapeBody.Joint = shapeJoint;
%     
%     % Add visual mesh - requires saving as stl first
%     visual_mesh = convertToCollisionMesh(cylinder);
%     K = convhull(visual_mesh.Vertices);
%     tri = triangulation(K, visual_mesh.Vertices);
%     fname = [fileparts(mfilename('fullpath')),'/panda_description/', 'custom_shape.stl'];
%     stlwrite(tri, fname)
%     addVisual(shapeBody, "Mesh", fname, T)
%     
%     % Add rigid body to robot model attached at panda_hand
%     addBody(panda_sc,shapeBody,"panda_hand");   
%     addBody(panda_ec,shapeBody,"panda_hand");   

    %% Assign collision and visual meshes for shape



end

