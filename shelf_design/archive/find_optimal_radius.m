% Pick the best radius for each shelf height. Run this after running
% find_max_radius.m

% Inputs copied from find_max_radius.m
num_theta_samples = 5;
theta_samples = linspace(-3*pi/4, 3*pi/4, num_theta_samples);
z_samples = -4*SHELF_SPACING:SHELF_SPACING:13*SHELF_SPACING;
num_z_samples = numel(z_samples);
radius_samples = 1:-0.01:0.35; % note descending
num_radius_samples = numel(radius_samples);

% Read result of find_max_radius.m
basefolder = pwd+"/saved_shape_positions/";
file_list = dir(basefolder);
for i = 1:numel(file_list)
    filename = file_list(i).folder+"/"+file_list(i).name;
end

% Choose largest radius