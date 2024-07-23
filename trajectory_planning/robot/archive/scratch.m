format long

addpath([pwd, '/../..'])
run parameters.m

r = 0.75;
x0 =  0;
y0 =  0;
z0 =  0.391;

%% Get sample path
% Inputs
start = [-0.0121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.04, 0.04];
goal = [-0.1121707,-0.561084,0.00127942,-1.60702,-0.0211893,2.03285,0.802306, 0.04,0.04];

% Load robot
addpath([pwd, '/../robot'])
% panda = loadPandaWithShape();

% Load collision environment
addpath([pwd, '/../collision_environment'])
env = build_collision_environment();
env = env(440:end);


%% Import robot
urdf = strcat(pwd,'/panda_description/panda.urdf');
rob = importrobot(urdf, 'DataFormat', 'row');

% Base
name = 'link0';
filename = strcat(pwd, '/panda_description/visual/', name, '.dae');
addVisual(rob.Base, 'Mesh',  filename)
filename = strcat(pwd, '/panda_description/collision/', name, '.stl');
addCollision(rob.Base, 'Mesh',  filename)

for i=1:11

    if i<=7
        name = strcat('link',num2str(i));
    elseif i==8
        continue
    elseif i==9
        name='hand';
    elseif i==10 || i==11
        name='finger';
    end

    % Get T
    if i==11
        T = eul2tform([pi, 0, 0]);
    else
        T = eye(4);
    end

    % Add visual mesh
    filename = strcat(pwd, '/panda_description/visual/', name, '.dae');
    addVisual(rob.Bodies{i}, 'Mesh',  filename, T)

    % Add collision mesh
    if strcmp(name, "finger")
%         addCollision(rob.Bodies{i}, 'cylinder',  [0.1, 0.1])
    elseif strcmp(name, "hand")
        assert(strcmp(rob.Bodies{i}.Name, strcat('panda_', name)))
        cylinder_length = 0.11; % large enough to also act as collision mesh for the two fingers
        addCollision(rob.Bodies{i}, "cylinder", [0.1, cylinder_length], trvec2tform([0.0, 0, cylinder_length/2]));
    else
        filename = strcat(pwd, '/panda_description/collision/', name, '.stl');
        addCollision(rob.Bodies{i}, 'Mesh',  filename)
    end
end
% Add a body 'ee' so that the forward kinematics align with the
% physical Panda's forward kinematics of the end effector.
extension_body = rigidBody('ee');
extension_joint = rigidBodyJoint('extension_joint', 'fixed');
setFixedTransform(extension_joint, [0,0,0.1033,0], 'dh')
extension_body.Joint = extension_joint;
addBody(rob, extension_body, 'panda_hand');

% Add Shape
collisionCylinderLength = 0.1;
collisionCylinderRadius = 0.025;
offset = 0.01;  % Prevent cylinder from being in collision with fingertips
cylinder = collisionCylinder(collisionCylinderRadius,collisionCylinderLength);
shapeBody = rigidBody("panda_custom_shape");
shapeJoint = rigidBodyJoint("shapeJoint");

% Place can into the end effector gripper.
setFixedTransform(shapeJoint,[0,0,collisionCylinderLength/2 + offset,0], 'dh'); 

% Add collision geometry to rigid body.
T = eye(4);
addCollision(shapeBody,cylinder,T);
shapeBody.Joint = shapeJoint;

%TODO: Add visual to canBody, requires .stl
visual_mesh = convertToCollisionMesh(cylinder);
K = convhull(visual_mesh.Vertices);
tri = triangulation(K, visual_mesh.Vertices);
fname = "custom_shape.stl";
stlwrite(tri, fname)
addVisual(shapeBody, "Mesh", "custom_shape.stl", T)
delete(fname)

% Add rigid body to robot model.
addBody(rob,shapeBody,"ee"); 

%TODO: Figure out why finger visuals are weird, located in wrong frame?
% show(rob, goal, 'Collision', 'off', 'Visuals', 'on'); hold on;
% [x,y,z] = sphere;
% r = 0.74;
% z0 = 0.39;
% x = x*r;
% y = y*r;
% z = z*r+z0;
% surf(x,y,z, 'EdgeColor','none','FaceAlpha',0.3,'FaceColor',[0.3010, 0.7450, 0.9330])
% % show(rob, start, 'Visuals', 'on', 'Collisions', 'on')
% hold on

%% Get points of collision bodies
% name = 'link5';
% xyz = stlread(strcat(pwd, "/panda_description/collision/", name, ".stl")).Points;
% tic
% xyzw = [xyz, ones(size(xyz,1),1)];  % homogenous coordinates;
% T = getTransform(rob, start, strcat('panda_', name))';
% xyz_T = xyzw*T;
% toc

% plot3(xyz_T(:,1),xyz_T(:,2),xyz_T(:,3), "r*" )

