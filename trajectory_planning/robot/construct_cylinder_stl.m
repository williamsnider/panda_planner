%% Load variables
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Make cylinders
for i = 1:4
savename = strcat("panda_description/cylinder",num2str(i),".stl");
make_cylinder(params.cylinderLength, params.cylinderRadius, savename);
end








