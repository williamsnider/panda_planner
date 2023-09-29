function env = build_collision_environment()
% Collision Environment for robot. Because it is challenging to align the
% experimental room to this model, the two may be misaligned by a small
% amount (I would estimate up to 1 inch). Be careful and don't rely 100% on
% this collision environment.

% Idea - to speed up the collision checking, drop all of the shelves, and
% instead test that the robot is within the largest ellipsoid that fits
% inside the shelves.

close all
collision_objects = cell(0);

%% Collision Shelves
dist_buffer = 0.0254;
dist_shelf_edge_to_peg_center= 0.0254/2 + 0.01 + dist_buffer;
box_length = 2*dist_shelf_edge_to_peg_center;
box_height = 0.11;

% Add additional values at end for max Z so that there is a second layer of
% boxes; this prevents the robot from entering the gap between the heighest
% and second highest shapes
Z_list = [-0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.1];
radius_list = [0.675, 0.765, 0.81, 0.835, 0.85, 0.85, 0.85, 0.83, 0.815, 0.775, 0.725, 0.645, 0.525, 0.29, 0.425 ];

th_range = [3*pi/4, -3*pi/4];
box_width = 0.2;


for j = 1:numel(Z_list)
    Z = Z_list(j);
    radius = radius_list(j);

    th_box = 2*atan(box_width/2/radius);
    num_boxes = ceil((th_range(1)-th_range(2))/th_box);
    th_vals = linspace(th_range(1), th_range(2), num_boxes);
    for i=1:numel(th_vals)
        
        % Get angle
        th = th_vals(i);
        
        % Convert to transformation matrix
        T = eye(4);
        T(1:3, 1:3) = eul2rotm([th,pi/2,0], "ZYX");   % pi/2 to have pose of box align with pose of robot needed to grasp shape (blue axis pointing away)
        T(1:2,4) = [radius*cos(th), radius*sin(th)];
        T(3,4) = Z;
    
    
        % Create box corresponding to shelf + object)
        box = collisionBox(box_length,box_width,box_height);
        box.Pose = T;
        collision_objects{end+1} = box;
    end
end

% NOTE! The collisions shelves have been replaced by sphere checking in the
% state space validator class

% json_string = fileread('collision_boxes.json');
% shelves_json = jsondecode(json_string);
% shelves_cell = struct2cell(shelves_json);
% num_shelves = numel(shelves_cell);
% for i=1:num_shelves
%     
%     shelf = shelves_cell{i};
%     T_flat = shelf.T;
%     T = reshape(T_flat, [4,4])';
%     
%     box = collisionBox(shelf.length,shelf.width,shelf.height);
%     box.Pose = T;
%     collision_objects{i} = box;
% end

%% Pedestal

% Four walls to avoid constant collision with bottom of robot (which is
% spherical)
ped_dims_in = [11,11,17];
ped_dims_m = ped_dims_in*0.0254;

% North 
box = collisionBox(ped_dims_m(1), 0.01, ped_dims_m(3));
T = trvec2tform([-1.5*0.0254,ped_dims_m(2)/2,-ped_dims_m(3)/2-0.001]);
box.Pose = T;
collision_objects{end+1} = box;

% South
box = collisionBox(ped_dims_m(1), 0.01, ped_dims_m(3));
T = trvec2tform([-1.5*0.0254,-ped_dims_m(2)/2,-ped_dims_m(3)/2-0.001]);
box.Pose = T;
collision_objects{end+1} = box;

% West 
box = collisionBox(0.01, ped_dims_m(2), ped_dims_m(3));
T = trvec2tform([ped_dims_m(1)/2-1.5*0.0254,0,-ped_dims_m(3)/2-0.001]);
box.Pose = T;
collision_objects{end+1} = box;

% East 
box = collisionBox(0.01, ped_dims_m(2), ped_dims_m(3));
T = trvec2tform([-ped_dims_m(1)/2-1.5*0.0254,0,-ped_dims_m(3)/2-0.001]);
box.Pose = T;
collision_objects{end+1} = box;

%% Cable Box
cable_dims_m = [6*0.0254, 5*0.0254, 0.1016+ped_dims_m(3)];
box = collisionBox(cable_dims_m(1), cable_dims_m(2), cable_dims_m(3));
T = trvec2tform([-1.5*0.0254, -ped_dims_m(2)/2-cable_dims_m(2)/2, -cable_dims_m(3)/2+0.1016]);
box.Pose = T;
collision_objects{end+1} = box;

%% Monkey reaching box
reach_dims_m = [0.01, 0.25, 0.3];
box = collisionBox(reach_dims_m(1), reach_dims_m(2), reach_dims_m(3));
T = trvec2tform([-0.55+reach_dims_m(1)/2,0.1,0.43]);
box.Pose = T;
collision_objects{end+1} = box;

%% Return environment
env = collision_objects;

%% Plot
% figHandle = figure;
% 
% % Show the first object
% [~, patchObj] = show(collision_objects{1});
% patchObj.FaceColor = [0 1 1];
% patchObj.EdgeColor = 'none';
% 
% % Get axis properties and set hold
% ax = gca;
% hold all;
% 
% % Show remaining objects
% for i = 2:numel(collision_objects)
%     [~, patchObj] = show(collision_objects{i}, "Parent", ax);
%     patchObj.FaceColor = [0 1 1];
%     patchObj.EdgeColor = 'none';
% end
% axis equal;

% robot = loadrobot("frankaEmikaPanda","DataFormat","column","Gravity",[0 0 -9.81]);
% 
% startConfig = [-1.94825142 -0.10557155 -0.58165625 -2.19966752 -1.26967682 1.02605133 1.42899875 0.01    0.01]';
% endConfig = [-0.9240    1.2703    1.9865    1.2394    1.7457   -2.0500    0.4222    0.01    0.01]';
% show(robot,startConfig,"Parent",ax, 'visuals', 'on', 'collision', 'off');
% 
% h = light;
% h.Style = 'infinite';
% h.Position = [-1, 0, 0];
% 
% lighting gouraud
% plotTransforms(se3(collision_objects{1}.Pose))

end
