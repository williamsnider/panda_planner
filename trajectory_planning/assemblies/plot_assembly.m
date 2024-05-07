function plot_assembly(panda_sc, q, triObj, cylinderRadius, XYZ)
% Color each object differently
s=show(panda_sc, q, 'Collisions', 'off','Frames',"off", 'Visuals', 'on');
h = findall(s, 'Type', 'patch');  % This finds all patch objects in the figure
colors = ['r', 'g', 'b', 'c', 'm'];
color_count = 1;
for i = numel(h):-1:1
    if all(size(h(i).Faces)==size(triObj.ConnectivityList))
        h(i).FaceColor = colors(color_count);
        h(i).EdgeColor='none';
        color_count = color_count +1;
    end
end

% Remove lines
hl = findall(s, 'Type', 'line');
delete(hl);




%% Plot reaching box

% Define the dimensions of the box
width = 0.15;  % X dimension
height = 0.05;  % Y dimension
length = 0.05;  % Z dimension

% Define vertices of the box in local coordinates (centered at the origin)
vertices = [
    -width/2, -height/2, 0;
    width/2, -height/2, 0;
    width/2, height/2, 0;
    -width/2, height/2, 0;
    -width/2, -height/2, length;
    width/2, -height/2, length;
    width/2, height/2, length;
    -width/2, height/2, length
];

% Define transformation matrix T
% Example: Move and rotate the box
T_box = eye(4);
% T_box(1:3,1:3) = ori_base;
T_box(1:3,4) = XYZ';
T_box(1,4) = T_box(1,4) -width/2 - 2*cylinderRadius;
T_box(3,4) = T_box(3,4) - height/2;

% Apply the transformation matrix to vertices
vertices = [vertices, ones(size(vertices, 1), 1)] * T_box';  % Homogeneous coordinates
vertices = vertices(:, 1:3);  % Drop homogeneous coordinate

% Center box


% Define faces of the box
faces = [
    1, 2, 6, 5;  % Front face
    2, 3, 7, 6;  % Right face
    3, 4, 8, 7;  % Back face
    4, 1, 5, 8;  % Left face
    1, 2, 3, 4;  % Bottom face
    5, 6, 7, 8   % Top face
];

% Plot the box using a single patch call
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'blue', 'EdgeColor', 'k');

%% Beautify figure
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([-1 0]);
ylim([-0.25 0.75]);
zlim([-0.25 0.75]);
view(-104, 40);
end

