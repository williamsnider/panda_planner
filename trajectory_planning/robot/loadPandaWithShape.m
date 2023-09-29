function [panda_ec, panda_sc] = loadPandaWithShape()
%LOADPANDAWITHSHAPE Loads the Franka Emika Panda with an attached shape
% 
% The robot is constructed from a urdf (as opposed to working with matlab's
% built-in model of the panda) because this exposes this collision files
% (e.g. link0.stl) which is utilized by ManipulatorRRTSphere to check if
% the manipuatlor is within a sphere (i.e. not colliding with the shelves).

    % Generate points in case anything was updated
    run generate_collision_point_lists.m

    %% Import robots (self collision and environment collision version)
    urdf_sc = strcat(fileparts(mfilename('fullpath')),'/panda_description/panda_sc.urdf');
    panda_sc = importrobot(urdf_sc, 'DataFormat', 'row' );
    
    urdf_ec = strcat(fileparts(mfilename('fullpath')),'/panda_description/panda_ec.urdf');
    panda_ec = importrobot(urdf_ec, 'DataFormat', 'row' );
    
%     disp("WARNING: Panda does not have a shape attached by default.")
    %% Assign collision and visual meshes for shape
    % prevents collisions between grasped shape and world

    % Create shape body/joint
    collisionCylinderLength = 0.1;
    collisionCylinderRadius = 0.025;
    offset = 0.01;  % Prevent cylinder from being in collision with fingertips
    cylinder = collisionCylinder(collisionCylinderRadius,collisionCylinderLength);
    shapeBody = rigidBody("panda_custom_shape");
    shapeJoint = rigidBodyJoint("shapeJoint");
    setFixedTransform(shapeJoint,[0,0,0.1034+collisionCylinderLength/2 + offset,0], 'dh');      % Place shape into the end effector gripper.

    % Add collision mesh
    T = eye(4);
    addCollision(shapeBody,cylinder,T);
    shapeBody.Joint = shapeJoint;
    
    % Add visual mesh - requires saving as stl first
    visual_mesh = convertToCollisionMesh(cylinder);
    K = convhull(visual_mesh.Vertices);
    tri = triangulation(K, visual_mesh.Vertices);
    fname = [fileparts(mfilename('fullpath')),'/panda_description/', 'custom_shape.stl'];
    stlwrite(tri, fname)
    addVisual(shapeBody, "Mesh", fname, T)
    
    % Add rigid body to robot model attached at panda_hand
    addBody(panda_sc,shapeBody,"panda_hand");   
    addBody(panda_ec,shapeBody,"panda_hand");   

    %% Assign collision and visual meshes for shape


end

