% Make and save struct containing link collision points
% This script needs to be run to update collision_mesh_points.mat, which
% ManipulatorStateValidatorSphere uses to check if the mesh is within a
% spherical constraint region.

% "custom_shape" mesh can be found in loadPandaWithShape script, except it
% is automatically deleted.

collision_mesh_points = struct();
name_list = ["link0", "link1", "link2", "link3", "link4", "link5", "link6", "link7", "hand", "custom_shape"];
for i = 1:numel(name_list)
    name = name_list(i);
    xyz = stlread(strcat(fileparts(mfilename('fullpath')), "/panda_description/", name, ".stl")).Points;
    collision_mesh_points.(name) = xyz;
end
savename = strcat(fileparts(mfilename('fullpath')), "/collision_mesh_points.mat");
save(savename, "collision_mesh_points")

