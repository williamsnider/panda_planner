% Code to find the best configurations for the staging area / monkey grasp
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Set random stream for reproducibility
stream = RandStream('mt19937ar', 'Seed', 123); 
RandStream.setGlobalStream(stream);


%% Load robot, restrict joint limits for safety
[panda_ec_no_quartet, panda_sc_no_quartet] = loadPandaWithShape(params);
[panda_ec_with_quartet, panda_sc_with_quartet] = loadPandaWithShape(params);

env = build_collision_environment();


% 
% fv_old = stlread('Holes_Vertical.stl');
% points_old = fv.Points;
% points_new = points_old *0.001; % Scale mm to m
% fv_new = triangulation(fv.ConnectivityList, points_new)
% stlwrite( fv_new,'Holes_Vertical_scaled.stl');


close all



% Remove old cylinders
remove_names = [];
for i = 1:numel(panda_ec_with_quartet.BodyNames)
    bodyName = panda_ec_with_quartet.BodyNames{i};
    
    % Check if the body contains "panda_cylinder"
    if contains(bodyName, 'panda_cylinder')
        remove_names = [remove_names; bodyName];
        % body = getBody(panda_ec_with_quartet, bodyName);
        
        % Remove all visuals by setting an empty visual mesh
    end
end

for i = 1:size(remove_names,1)
    bodyName_to_remove = remove_names(i, :);
    removeBody(panda_ec_with_quartet, bodyName_to_remove)
    removeBody(panda_ec_no_quartet, bodyName_to_remove)
end


%% Attach stl to robot
% Create shape body/joint
collisionCylinderLength = 0.1;
collisionCylinderRadius = 0.025;
offset = 0.01;  % Prevent cylinder from being in collision with fingertips
cylinder = collisionCylinder(collisionCylinderRadius,collisionCylinderLength);
shapeBody = rigidBody("panda_custom_shape");
shapeJoint = rigidBodyJoint("shapeJoint");
setFixedTransform(shapeJoint,[0,0,0.1034+collisionCylinderLength/2 + offset,0], 'dh');      % Place shape into the end effector gripper.

% Add collision mesh
T = eye(4);  % Identity transformation
addCollision(shapeBody,cylinder,T);
shapeBody.Joint = shapeJoint;

% Add visual mesh - requires saving as stl first
fname = "D005_scaled.stl";
T_visual = makehgtform('zrotate', pi); % 180° rotation about zaxis
T_visual(3,4) = T_visual(3,4) + 0.005;
addVisual(shapeBody, "Mesh", fname, T_visual)

% Add rigid body to robot model attached at panda_hand
addBody(panda_sc_with_quartet,shapeBody,"panda_hand");   
addBody(panda_ec_with_quartet,shapeBody,"panda_hand");   



% 
% %% Extreme Positions
% mat_fname = strcat(params.CustomParametersDir,'/trajectory_planning/quartets/trajectories/20241009_A.mat');
% data_struct = load(mat_fname, 'data_struct');
% data_struct = data_struct.data_struct;
% 
% 
% 
% 
% 
% 
% % A0 B0 C0 D0
% close all
% idx_list = [1,9, 17, 25]+1;
% idx_list = [2,10,18,26]; % A0b, B0b, C0b, D0b
% idx_list= [idx_list, 25,27,29,31]; % D0a, D1a, D2a, D3a
% % idx_list = [1];
% for i = 1:numel(idx_list)
%     idx = idx_list(i);
% 
%     % Plot robot
%     q = data_struct.extreme_arr(idx,:);
%     g = show(panda_ec_with_quartet, q, 'Frames', 'off');
% 
% 
%     % Improve visibility
%     g.Children(2).FaceColor = 'g';
% 
%     % Add holes vertical
%     fv = stlread('Holes_Vertical_scaled.stl');
%     vertices = fv.Points;
%     faces = fv.ConnectivityList;
%     T_vh = eye(4);
%     xyz_old = (min(fv.Points,[],1)+max(fv.Points,[],1))/2;
%     xyz_new = getTransform(panda_ec_with_quartet, data_struct.extreme_arr(1,:), "panda_hand_tcp");
%     xyz_new = xyz_new(1:3,4);
%     T_vh(1:3, 4) = (xyz_new - xyz_old');
%     T_vh(2,4) = T_vh(2,4) - 0.075*3/2;
%     T_vh(1,4) = T_vh(1,4) - 0.0175;
%     numPts = size(vertices, 1);
%     vertices_h = [vertices, ones(numPts, 1)]'; % Convert to (4xN)
%     transformed_vertices_h = T_vh * vertices_h; 
%     transformed_vertices = transformed_vertices_h(1:3, :)';
%     vertices = transformed_vertices;
%     h = patch('Faces', faces, 'Vertices', vertices, ...
%               'FaceColor', 'b', 'EdgeColor', 'none', ...
%            'AmbientStrength', 0.3, ...
%            'FaceAlpha', 0.5);
% 
% 
% 
%     set(gca, 'CameraPosition', [-17.174195025218289  15.560687643254724  11.741204843223604], ...
%          'CameraTarget', [-0.004464113626022  -0.252876134488346   0.333683593155064], ...
%          'CameraUpVector', [0 0 1], ...
%          'CameraViewAngle', 1.832966159414962);
% 
%     grid off
%     camlight(-15,-45) % Light follows the camera
% 
%     set(gcf, 'WindowState', "maximized")
%     filename = sprintf('figure_whackamole_%d.png', i);
% 
%     % Pause on first loop
%     if i == 1
%         pause(3)
%     end
%     print(gcf, filename, '-dpng', '-r600'); % Saves full figure at 600 DPI
% %     waitforbuttonpress()
% end
% 
% 
% 
% %% UpOut etc Positions
% close all;
% fname = params.CustomParametersDir+"/trajectory_planning/slots/trajectories/20241009_06A36_downIn_to_downOut_40%.csv";
% arr = readmatrix(fname);
% q_downIn = [arr(1,:), 0.05, 0.05];
% q_downOut = [arr(end,:),  0.05, 0.05];
% fname = fname.replace("downIn_to_downOut", "upIn_to_upOut");
% arr = readmatrix(fname);
% q_upIn = [arr(1,:),  0.05, 0.05];
% q_upOut = [arr(end,:),  0.05, 0.05];
% 
% 
% 
% q_list = [q_downOut; q_downIn; q_upIn; q_upOut];
% robot_list = [panda_ec_no_quartet, panda_ec_no_quartet, panda_ec_with_quartet, panda_ec_with_quartet];
% shape_list = [1, 1, 0, 0];
% 
% % Pick
% close all;
% for i = 1:size(q_list,1)
% 
%     q = q_list(i,:);
%     robot = robot_list(i);
%     show_shape = shape_list(i);
%     
%     % Plot robot
%     g=show(robot, q, 'Frames', 'off'); hold on;
% 
%     % Color quartet if present
%     if robot == panda_ec_with_quartet
%         g.Children(2).FaceColor = 'g';
%     end
% 
%     % Plot shape
%     if show_shape == 1
%         fv = stlread('D005_scaled.stl');
%         vertices = fv.Points;
%         faces = fv.ConnectivityList;
%         numPts = size(vertices, 1);
%         vertices_h = [vertices, ones(numPts, 1)]'; % Convert to (4xN)
%         T_downIn = getTransform(panda_ec_with_quartet, q_downIn, 'panda_custom_shape') *  makehgtform('zrotate', pi);
%         T_downIn(1:3,4) = T_downIn(1:3,4) + T_downIn(1:3,3)*0.005;
%         transformed_vertices_h = T_downIn * vertices_h; 
%         transformed_vertices = transformed_vertices_h(1:3, :)';
%         vertices = transformed_vertices;
%         h = patch('Faces', faces, 'Vertices', vertices, ...
%                   'FaceColor', 'g', 'EdgeColor', 'none');
%     end
%     
%     
%     % Beautify
%     grid off
%     set(gca, 'CameraPosition', [23.588016815690292 -10.771172394345308   0.902157518594729], ...
%      'CameraTarget', [0.049474782398036   0.213334511356998   0.375235336946964], ...
%      'CameraUpVector', [0 0 1], ...
%      'CameraViewAngle', 1.484746135773886);
% %     camlight(-15,-45) % Light follows the camera
%     set(gcf, 'WindowState', "maximized")
%     filename = sprintf('figure_pickplace_%d.png', i);
%     print(gcf, filename, '-dpng', '-r600'); % Saves full figure at 600 DPI
% %     waitforbuttonpress()
%     hold off;
% end
% 
% 
% % Pick
% show(panda_ec_no_quartet, q_downOut, 'Frames', 'off'); hold on;
% % Plot shape alone
% 
% % Plot shape alone
% 
% fv = stlread('D005_scaled.stl');
% 
% % Extract vertices and faces
% vertices = fv.Points;
% faces = fv.ConnectivityList;
% 
% % Transform vertices
% numPts = size(vertices, 1);
% vertices_h = [vertices, ones(numPts, 1)]'; % Convert to (4xN)
% transformed_vertices_h = T_downIn * vertices_h; 
% transformed_vertices = transformed_vertices_h(1:3, :)';
% vertices = transformed_vertices;
% 
% % Plot using patch
% c = [78 165 96]/255;
% h = patch('Faces', faces, 'Vertices', vertices, ...
%           'FaceColor', 'g', 'EdgeColor', 'none');
% 
% 
% 
% 
% 
% show(panda_ec_with_quartet, q_downIn, 'Frames', 'off');
% hold on;
% show(panda_ec_with_quartet, q_downOut, 'Frames', 'off');
% show(panda_ec_with_quartet, q_upIn, 'Frames', 'off');
% show(panda_ec_with_quartet, q_upOut, 'Frames', 'off');
% 
% g = gca();
% g.Children(2).FaceColor = 'g';
% 
% 
% 
% 
% %% Motions
% close all;
% 
% % Cartesian Motion
% num_frames = 3;
% fname = params.CustomParametersDir+"/trajectory_planning/slots/trajectories/20241009_06A36_downOut_to_downIn_70%.csv";
% arr = readmatrix(fname);
% arr = [arr, 0.01 * ones(size(arr,1),2)];
% 
% % Find T at each q
% T_arr = zeros(size(arr,1),4,4);
% for i = 1:size(arr,1)
%     T = getTransform(panda_sc_no_quartet, arr(i,:), "panda_hand_tcp");
%     T_arr(i,:,:) = T;
% end
% 
% % Find T evenly split between start and end
% T_even = transformtraj(squeeze(T_arr(1,:,:)), squeeze(T_arr(end,:,:)), [0,1],linspace(0,1,num_frames));
% 
% % Find closest T_arr for each T_even
% frame_timesteps = [];
% 
% for e_num = 1:size(T_even,3)
%     T_e = T_even(:,:,e_num);
%     
%     min_dist = Inf;
%     best_index = -1;
%     
%     for a_num = 1:size(T_arr,1)
%         T_a = squeeze(T_arr(a_num,:,:));
%         
%         pair_dist = sum((T_a(1:3,4) - T_e(1:3,4)).^2);
%         
%         if pair_dist < min_dist
%             min_dist = pair_dist;
%             best_index = a_num; % Store the best index
%         end
%     end
%     
%     frame_timesteps = [frame_timesteps, best_index];
% end
% 
% num_shades = num_frames; % Number of shades to darken over time
% existing_patches = []; % Keep track of patches
% 
% blue_vals = linspace(1,4/6,num_shades);
% alpha_shades = linspace(0.25,1, num_shades);
% for i = 1:num_frames
%     % Show the current frame
%     show(panda_ec_no_quartet, arr(frame_timesteps(i),:), "Frames", "off");
%     hold on;
% 
%     % Get all patch objects AFTER adding the new ones
%     all_patches = findobj(gca, 'Type', 'Patch');
% 
%     % Identify newly added patches
%     new_patches = setdiff(all_patches, existing_patches);
%     
%     % Compute the shade of blue (lighter to darker over iterations)
% %     shade_factor = i / num_shades; % Starts light, becomes dark
% %     new_color = [0.8 0.8 blue_vals(i)]; % Light blue to dark blue
%     new_color = [0.8,0.8 0.4];
% 
%     % Apply the new color only to the newly added patches
%     for p = 1:length(new_patches)
% %         new_patches(p).FaceColor = new_color;
%         new_patches(p).FaceAlpha = alpha_shades(i);
%     end
% 
%     % Update the list of existing patches
%     existing_patches = all_patches;
% end
% hold off;
% 
% 
% % Plot shape
% fv = stlread('D005_scaled.stl');
% vertices = fv.Points;
% faces = fv.ConnectivityList;
% numPts = size(vertices, 1);
% vertices_h = [vertices, ones(numPts, 1)]'; % Convert to (4xN)
% T_downIn = getTransform(panda_ec_with_quartet, q_downIn, 'panda_custom_shape') *  makehgtform('zrotate', pi);
% T_downIn(1:3,4) = T_downIn(1:3,4) + T_downIn(1:3,3)*0.005;
% transformed_vertices_h = T_downIn * vertices_h; 
% transformed_vertices = transformed_vertices_h(1:3, :)';
% vertices = transformed_vertices;
% h = patch('Faces', faces, 'Vertices', vertices, ...
%           'FaceColor', 'g', 'EdgeColor', 'none');
%     
% grid off
% set(gca, 'CameraPosition', [-16.7298   17.1829    9.6006], ...
%  'CameraTarget', [0.3408   -0.0127    0.2247], ...
%  'CameraUpVector', [0 0 1], ...
%  'CameraViewAngle', 1.8330);
% camlight(-15,-45) % Light follows the camera
% set(gcf, 'WindowState', "maximized")
% pause(3)
% filename = sprintf('figure_cartesian_%d.png', i);
% print(gcf, filename, '-dpng', '-r600'); % Saves full figure at 600 DPI
% hold off;
% 
% 
% 
%% Joint Motion
close all;
num_frames = 3;
alpha_shades = linspace(0.25,1, num_frames);
fname = params.CustomParametersDir+"/trajectory_planning/slots/trajectories/20241009_06C08_upOut_to_stagingA0a_70%.csv";
arr = readmatrix(fname);
arr = [arr, 0.01 * ones(size(arr,1),2)];

%% Trim ends to get clear joint-to-joint
% plot_derivatives(arr(:,1:7),params)
arr = arr(825:3513,:);


% Find T at each q
T_arr = zeros(size(arr,1),4,4);
for i = 1:size(arr,1)
    T = getTransform(panda_sc_no_quartet, arr(i,:), "panda_hand_tcp");
    T_arr(i,:,:) = T;
end

frame_timesteps = round(linspace(1,size(arr,1),num_frames));
existing_patches = []; % Keep track of patches
for i = 1:num_frames
    % Show the current frame
    show(panda_ec_with_quartet, arr(frame_timesteps(i),:), "Frames", "off");
    hold on;

    % Get all patch objects AFTER adding the new ones
    all_patches = findobj(gca, 'Type', 'Patch');

    % Identify newly added patches
    new_patches = setdiff(all_patches, existing_patches);

    % Apply the new color only to the newly added patches
    for p = 1:length(new_patches)
        new_patches(p).FaceAlpha = alpha_shades(i);
    end

    % Update the list of existing patches
    existing_patches = all_patches;
end

g = color_quartet(gca(), size(vertices,1));


grid off
set(gca, 'CameraPosition', [-1.6794   -0.0695   25.9313], ...
 'CameraTarget', [0.0251   -0.0821    0.0065], ...
 'CameraUpVector', [0 0 1], ...
 'CameraViewAngle', 2.2628);
% camlight(-15,-45) % Light follows the camera
set(gcf, 'WindowState', "maximized")
pause(1)

plot3(squeeze(T_arr(:,1,4)), squeeze(T_arr(:,2,4)), squeeze(T_arr(:,3,4)), ...
      'm-', 'LineWidth', 5, 'Tag', 'trajectory');
filename = sprintf('figure_joint_%d.png', 1);
print(gcf, filename, '-dpng', '-r600'); % Saves full figure at 600 DPI

% Write arr to csv
filename = "joint_data.csv";
writematrix(arr, filename);






%% Collision Environment
close all;
plotJointMotion(panda_ec_with_quartet, params.q_home, env, params)

fv = stlread('D005_scaled.stl');
vertices = fv.Points;
g = gca();
g = color_quartet(g, size(vertices,1));
grid off;
axis off;
set(gcf, 'Color', 'w'); % 'w' stands for white
% camlight(-45, -45); % Adjust light direction
lightHandle = camlight('right'); % Create light
set(lightHandle, 'Color', [0.5, 0.5, 0.5]); % Reduce brightness by setting a dimmer light color
set(gca, 'CameraPosition', [-12.6981   21.6923    6.5866], ...
 'CameraTarget', [ -0.0572   -0.1517    0.4182], ...
 'CameraUpVector', [0 0 1], ...
 'CameraViewAngle',   3.8312);
set(gcf, 'WindowState', "maximized")
pause(1)
filename = sprintf('figure_collision_environment_%d.png', 1);
print(gcf, filename, '-dpng', '-r600'); % Saves full figure at 600 DPI
