function plotJointMotion(panda, anglesArray, collisionObjectArray, params)

sphere_radius = params.sphere_radius;
sphere_origin = params.sphere_origin;

%PLOTJOINTMOTION Plots robot given angles
%   Detailed explanation goes here
% Display
controlHz = 1000;
dispHz = 15;

% Show panda first to get correct plot zoom
% show(panda, anglesArray(1,:), 'Visuals','on','Collisions', 'off',  'Frames', 'off');

% Get axis properties and set hold
show(panda, anglesArray(1,:))
ax = gca;
hold all;







rateCtrlObj = rateControl(dispHz);
shortAnglesArray = anglesArray(1:round(controlHz/dispHz):end, :);
for i=1:size(shortAnglesArray, 1)
    show(panda, shortAnglesArray(i,:), "FastUpdate", true, "PreservePlot",false,"Visuals", "on", "Collisions", "off", 'Frames', 'off');
    if i == 1
        view(-134, 30)

                % Show remaining objects
        for i = 1:numel(collisionObjectArray)
            [~, patchObj] = show(collisionObjectArray{i}, "Parent", ax);
            patchObj.FaceColor = [0 1 1];
            patchObj.EdgeColor = 'none';
        end

        % Show collision sphere
        if exist('params','var')
            [X,Y,Z] = sphere(100);
            X = X*sphere_radius ;
            Y = Y*sphere_radius ;
            Z = Z*sphere_radius + sphere_origin(3);

            % Cull Z above/below
            mask = (Z>params.sphere_cutoff_bottom) & (Z<params.sphere_cutoff_top);
            [r,c] = find(mask);
            unique_r = unique(r);
            X = X(r(1):r(end),:);
            Y = Y(r(1):r(end),:); 
            Z = Z(r(1):r(end),:);

            h=surf(X,Y,Z);
            set(h, 'FaceAlpha',0.25, 'FaceColor', [0,1,1],'edgecolor','none')

            % Plot cylinders
            x = linspace(0, 2*pi, 20);
            r = params.bottom_cylinder_radius;
            X = [r*cos(x); r*cos(x)];
            Y = [r*sin(x); r*sin(x)]; 
            Z = [-params.cylinder_height/2*ones(size(x));params.cylinder_height/2*ones(size(x)) ] + params.sphere_cutoff_bottom;
            h=surf(X,Y,Z);
            set(h, 'FaceAlpha',0.25, 'FaceColor', [0,1,1],'edgecolor','none')            

            % Plot cylinders
            r = params.top_cylinder_radius;
            X = [r*cos(x); r*cos(x)];
            Y = [r*sin(x); r*sin(x)]; 
            Z = [-params.cylinder_height/2*ones(size(x));params.cylinder_height/2*ones(size(x)) ] + params.sphere_cutoff_top;
            h=surf(X,Y,Z);
            set(h, 'FaceAlpha',0.25, 'FaceColor', [0,1,1],'edgecolor','none')            
            axis equal


        end

    end
    waitfor(rateCtrlObj);
end
end

