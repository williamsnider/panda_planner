function plotJointMotion(panda, anglesArray, collisionObjectArray, params)

sphere_radius = params.sphere_radius;
sphere_origin = params.sphere_origin;

%PLOTJOINTMOTION Plots robot given angles
%   Detailed explanation goes here
% Display
controlHz = 1000;
dispHz = 10;


% Handle anglesArray Nx7 or Nx9
[num_rows, num_cols] = size(anglesArray);
if num_cols == 7
    anglesArray = [anglesArray, 0.01*ones(num_rows,2)];
elseif num_cols ~= 9
    error('Trajectory array does not have 7 or 9 columns.');
end


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



    zone_color = [0,1,1];
    zone_alpha = 0.25;
    collision_color = 'b';%[0.8,0.1,0.1];
    collision_alpha = 1.0;

    fv = load_uniform_zone_mesh(params);

    rateCtrlObj = rateControl(dispHz);
    shortAnglesArray = anglesArray(1:round(controlHz/dispHz):end, :);
    for i=1:size(shortAnglesArray, 1)
        show(panda, shortAnglesArray(i,:), "FastUpdate", true, "PreservePlot",false,"Visuals", "on", "Collisions", "off", 'Frames', 'off');
        if i == 1
            view(-134, 30)

            % Show remaining objects
            for j = 1:numel(collisionObjectArray)
                [~, patchObj] = show(collisionObjectArray{j}, "Parent", ax);
                patchObj.FaceColor = collision_color;
                patchObj.EdgeColor = 'none';
                patchObj.FaceAlpha = collision_alpha;
            end

            %         % Show collision sphere
            %         if exist('params','var')
            %             [X,Y,Z] = sphere(100);
            %             X = X*sphere_radius ;
            %             Y = Y*sphere_radius ;
            %             Z = Z*sphere_radius + sphere_origin(3);
            %
            %             % Cull Z above/below
            %             SPHERE_CYLINDER_OVERLAP_FOR_PLOTTING = 0.05;
            %             mask = (Z>params.sphere_cutoff_bottom+params.cylinder_height/2-SPHERE_CYLINDER_OVERLAP_FOR_PLOTTING) & (Z<params.sphere_cutoff_top);
            %             [r,c] = find(mask);
            %             unique_r = unique(r);
            %             X = X(r(1):r(end),:);
            %             Y = Y(r(1):r(end),:);
            %             Z = Z(r(1):r(end),:);
            %
            %             % ====== REMOVE SPHERE PATCHES INSIDE BOX ======
            %             % Define the bounding box limits
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
            %
            %
            %             box_min = staging_box_center - [halfWidth, halfLength, halfHeight];
            %             box_max = staging_box_center + [halfWidth, halfLength, halfHeight];
            %
            %             % Create mask for sphere points that are OUTSIDE the box
            %             mask_outside_box = (X < box_min(1) | X > box_max(1)) | ...
            %                                (Y < box_min(2) | Y > box_max(2)) | ...
            %                                (Z < box_min(3) | Z > box_max(3));
            %
            %             % Apply mask to remove sphere portions inside the box
            %             X(~mask_outside_box) = NaN;
            %             Y(~mask_outside_box) = NaN;
            %             Z(~mask_outside_box) = NaN;
            %
            %             h=surf(X,Y,Z);
            %             set(h, 'FaceAlpha',zone_alpha, 'FaceColor', zone_color,'edgecolor','none')
            %
            %             % Plot cylinders
            %             x = linspace(0, 2*pi, 50);
            %             r = params.bottom_cylinder_radius;
            %             X = [r*cos(x); r*cos(x)];
            %             Y = [r*sin(x); r*sin(x)];
            %             Z = [-params.cylinder_height/2*ones(size(x));params.cylinder_height/2*ones(size(x)) ] + params.sphere_cutoff_bottom;
            %             h=surf(X,Y,Z);
            %             set(h, 'FaceAlpha',zone_alpha, 'FaceColor', zone_color,'edgecolor','none')
            %
            %             % Plot cylinders - top cylinder, only plot top portion since it
            %             % is contained by sphere.
            %             r = params.top_cylinder_radius;
            %             X = [r*cos(x); r*cos(x)];
            %             Y = [r*sin(x); r*sin(x)];
            % %             Z = [-params.cylinder_height/2*ones(size(x));params.cylinder_height/2*ones(size(x)) ] + params.sphere_cutoff_top;
            %             Z = [-SPHERE_CYLINDER_OVERLAP_FOR_PLOTTING*ones(size(x));params.cylinder_height/2*ones(size(x)) ] + params.sphere_cutoff_top;
            %             h=surf(X,Y,Z);
            %             set(h, 'FaceAlpha',zone_alpha, 'FaceColor', zone_color,'edgecolor','none')
            %             axis equal
            %
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

            h = patch('Vertices', fv.Points, 'Faces', fv.ConnectivityList,   'FaceColor',zone_color, 'FaceAlpha', zone_alpha, 'EdgeColor', 'none');


        end

    end
    waitfor(rateCtrlObj);
end
end

