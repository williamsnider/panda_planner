function make_cylinder(length, radius, savename )

% Create the cylinder
num_samples = 20;
R = [1,1];
[X, Y, Z] = cylinder(R,num_samples);

num_points = 2*(num_samples+1);

% Convert surface data to patch data
FV = surf2patch(X, Y, Z, 'triangles');

% Insert bottom and top center points
FV.vertices = [ FV.vertices; [0,0,0]; [0,0,1]];

% Link bottom faces
new_faces = [];

for i = 1:2:num_points
    
    v1 = 43;
    v2 = mod(i, num_points);
    v3 = mod(i+2, num_points);

    new_faces = [new_faces;[v1, v2, v3]];
end


for i = 2:2:num_points
    
    v1 = 44;
    v2 = i;
    v3 = i+2;

    if v2 > num_points
        v2 = mod(v2,num_points);
    end


    if v3 > num_points
        v3 = mod(v3, num_points);
    end

    

    new_faces = [new_faces;[v1, v2, v3]];
end

FV.faces = [FV.faces; new_faces];


% Adjust length and radius
FV.vertices(:,1) = FV.vertices(:,1)*radius;
FV.vertices(:,2) = FV.vertices(:,2)*radius;
FV.vertices(:,3) = FV.vertices(:,3)*length ;  


% Create a triangulation object
TR = triangulation(FV.faces, FV.vertices);

% Save as STL
stlwrite(TR, savename);