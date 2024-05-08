function plotJointMotion(panda, anglesArray, collisionObjectArray, params)

sphere_radius = params.sphere_radius;
sphere_origin = params.sphere_origin;

%PLOTJOINTMOTION Plots robot given angles
%   Detailed explanation goes here
% Display
controlHz = 1000;
dispHz = 10;

% Show panda first to get correct plot zoom
% show(panda, anglesArray(1,:), 'Visuals','on','Collisions', 'off',  'Frames', 'off');

% Test if plot already contains robot
hasArm = false;
children = get(gca,'Children');
for i = 1:numel(children)
       if strcmp(get(children(i), 'Type'), 'Transform')
            hasArm = true;
            break;
       end
end

if ~hasArm
    % Get axis properties and set hold
    show(panda, anglesArray(1,:), "FastUpdate", true, "PreservePlot",false,"Visuals", "on", "Collisions", "off", 'Frames', 'off');
    ax = gca;
hold all;







rateCtrlObj = rateControl(dispHz);
shortAnglesArray = anglesArray(1:round(controlHz/dispHz):end, :);
for i=1:size(shortAnglesArray, 1)
    show(panda, shortAnglesArray(i,:), "FastUpdate", true, "PreservePlot",false,"Visuals", "on", "Collisions", "off", 'Frames', 'off');
    if i == 1
        view(-134, 30)

                % Show remaining objects
        for j = 1:numel(collisionObjectArray)
            [~, patchObj] = show(collisionObjectArray{j}, "Parent", ax);
            patchObj.FaceColor = [0 1 1];
            patchObj.EdgeColor = 'none';
        end
% 
%         % Show collision sphere
%         if exist('params','var')
%             [X,Y,Z] = sphere(100);
%             X = X*sphere_radius ;
%             Y = Y*sphere_radius ;
%             Z = Z*sphere_radius + sphere_origin(3);
% 
%             % Cull Z above/below
%             mask = (Z>params.sphere_cutoff_bottom) & (Z<params.sphere_cutoff_top);
%             [r,c] = find(mask);
%             unique_r = unique(r);
%             X = X(r(1):r(end),:);
%             Y = Y(r(1):r(end),:); 
%             Z = Z(r(1):r(end),:);
% 
%             h=surf(X,Y,Z);
%             set(h, 'FaceAlpha',0.25, 'FaceColor', [0,1,1],'edgecolor','none')
% 
%             % Plot cylinders
%             x = linspace(0, 2*pi, 20);
%             r = params.bottom_cylinder_radius;
%             X = [r*cos(x); r*cos(x)];
%             Y = [r*sin(x); r*sin(x)]; 
%             Z = [-params.cylinder_height/2*ones(size(x));params.cylinder_height/2*ones(size(x)) ] + params.sphere_cutoff_bottom;
%             h=surf(X,Y,Z);
%             set(h, 'FaceAlpha',0.25, 'FaceColor', [0,1,1],'edgecolor','none')            
% 
%             % Plot cylinders
%             r = params.top_cylinder_radius;
%             X = [r*cos(x); r*cos(x)];
%             Y = [r*sin(x); r*sin(x)]; 
%             Z = [-params.cylinder_height/2*ones(size(x));params.cylinder_height/2*ones(size(x)) ] + params.sphere_cutoff_top;
%             h=surf(X,Y,Z);
%             set(h, 'FaceAlpha',0.25, 'FaceColor', [0,1,1],'edgecolor','none')            
%             axis equal
% 
%             % Plot box on non-shelved side of sphere
%             staging_box_width = params.staging_box_width;
%             staging_box_height = params.staging_box_height;
%             staging_box_length = params.staging_box_length;
%             staging_box_center = params.staging_box_center;
% 
%             % Compute box corners based on the center point
%             halfWidth = staging_box_width / 2;
%             halfHeight = staging_box_height / 2;
%             halfLength = staging_box_length / 2;
%             
%             % Define the 8 vertices of the box
%             vertices = [
%                 staging_box_center + [-halfWidth, -halfLength, -halfHeight];
%                 staging_box_center + [halfWidth, -halfLength, -halfHeight];
%                 staging_box_center + [halfWidth, halfLength, -halfHeight];
%                 staging_box_center + [-halfWidth, halfLength, -halfHeight];
%                 staging_box_center + [-halfWidth, -halfLength, halfHeight];
%                 staging_box_center + [halfWidth, -halfLength, halfHeight];
%                 staging_box_center + [halfWidth, halfLength, halfHeight];
%                 staging_box_center + [-halfWidth, halfLength, halfHeight];
%             ];
%             
%             % Define the faces of the box using vertex indices
%             faces = [
%                 1, 2, 3, 4; % Bottom face
%                 5, 6, 7, 8; % Top face
%                 1, 2, 6, 5; % Front face
%                 2, 3, 7, 6; % Right face
%                 3, 4, 8, 7; % Back face
%                 4, 1, 5, 8; % Left face
%             ];
%             
%             h = patch('Vertices', vertices, 'Faces', faces,   'FaceColor', [0, 1, 1], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
% 

%         end

    end
    waitfor(rateCtrlObj);
end
end

