function [env_norm, env_big] = build_collision_environment()
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
ped_width = 0.24;  % Left-to-right
ped_height = 17*0.0254;  % 17 inches
ped_depth = 0.31;
% ped_depth = 0.012 % Northside
% ped_depth = 0.019 % Southside
X_shift = -0.035;
% ped_dims_in = [11,11,17];
% ped_dims_m = [11*0.0254, 0.24, 17*0.0254];

% North 
box = collisionBox(ped_depth, 0.01, ped_height);
T = trvec2tform([X_shift,ped_width/2,-ped_height/2-0.001]);
box.Pose = T;
collision_objects{end+1} = box;

% South
box = collisionBox(ped_depth, 0.01, ped_height);
T = trvec2tform([X_shift,-ped_width/2,-ped_height/2-0.001]);
box.Pose = T;
collision_objects{end+1} = box;

% West 
box = collisionBox(0.01, ped_width, ped_height);
T = trvec2tform([ped_depth/2+X_shift,0,-ped_height/2-0.001]);
box.Pose = T;
collision_objects{end+1} = box;

% East 
box = collisionBox(0.01, ped_width, ped_height);
T = trvec2tform([-ped_depth(1)/2+X_shift,0,-ped_height/2-0.001]);
box.Pose = T;
collision_objects{end+1} = box;

%% Cable Boxes
cable_dims_m = [5*0.0254, 4*0.0254, 2*0.1016];
box = collisionBox(cable_dims_m(1), cable_dims_m(2), cable_dims_m(3));
T = trvec2tform([X_shift, -ped_width/2-cable_dims_m(2)/2, 0]);
box.Pose = T;
collision_objects{end+1} = box;

box = collisionBox(cable_dims_m(1), cable_dims_m(2), cable_dims_m(3));
T = trvec2tform([-1.5*0.0254, ped_width/2+cable_dims_m(2)/2, 0]);
box.Pose = T;
collision_objects{end+1} = box;

%% Monkey reaching box
box_dims_m = [0.1, 0.1, 0.1];
box_trans_m = [-0.62,0.175, 0.5526];
box = collisionBox(box_dims_m(1), box_dims_m(2), box_dims_m(3));
T = trvec2tform(box_trans_m);
box.Pose = T;
collision_objects{end+1} = box;


%% Monkey chair
chair_dims_m = [0.01, 1.0, 1.0];
chair_trans_m = [-0.7, 0, 0.2];
box = collisionBox(chair_dims_m(1), chair_dims_m(2), chair_dims_m(3));
T = trvec2tform(chair_trans_m);
box.Pose = T;
collision_objects{end+1} = box;


%% Return environment
env_norm = collision_objects;

%% Return environment expanded slightly
big_factor = 1.1;
env_big = cell(0);
for box_num = 1:numel(env_norm)
    box_norm = env_norm{box_num};

    box_big = collisionBox(box_norm.X*big_factor, box_norm.Y*big_factor, box_norm.Z*big_factor);
    box_big.Pose = box_norm.Pose;
    env_big{end+1} = box_big;
end

%% Plot
% % figHandle = figure;
% % 
% % Show the first object
% % [~, patchObj] = show(collision_objects{1});
% % patchObj.FaceColor = [0 1 1];
% % patchObj.EdgeColor = 'none';
% % 
% % Get axis properties and set hold
% % ax = gca;
% % hold all;
% % 
% % Show remaining objects
% % for i = 2:numel(collision_objects)
% %     [~, patchObj] = show(collision_objects{i}, "Parent", ax);
% %     patchObj.FaceColor = [0 1 1];
% %     patchObj.EdgeColor = 'none';
% % end
% % axis equal;
% % 
% % robot = loadrobot("frankaEmikaPanda","DataFormat","column","Gravity",[0 0 -9.81]);
% % 
% % startConfig = [-1.94825142 -0.10557155 -0.58165625 -2.19966752 -1.26967682 1.02605133 1.42899875 0.01    0.01]';
% % endConfig = [-0.9240    1.2703    1.9865    1.2394    1.7457   -2.0500    0.4222    0.01    0.01]';
% % show(robot,startConfig,"Parent",ax, 'visuals', 'on', 'collision', 'off');
% % 
% % h = light;
% % h.Style = 'infinite';
% % h.Position = [-1, 0, 0];
% % 
% % lighting gouraud

end
