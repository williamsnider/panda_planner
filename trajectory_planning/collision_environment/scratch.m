%% Clear all variables and load parameters
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

%% Load points
pts = params.shelf_pts;

%% Calculate ellipsoid
[optimal_radii, optimal_center_Z, ~, ~, adjusted_pts] = fit_ellipsoid_to_shelves(params);

%% Plot ellipsoid as patch
hold on;
plot3(pts(:,1), pts(:,2), pts(:,3), "k*")

% Ellipsoid parameters
a = optimal_radii(1);
b = optimal_radii(2);
c = optimal_radii(3);
z0 = optimal_center_Z;

% Generate ellipsoid surface
[x, y, z] = ellipsoid(0, 0, z0, a, b, c, 40);
surf(x, y, z, 'FaceAlpha', 0.3, 'EdgeColor', 'none')

% Axis settings
axis equal
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Fitted Ellipsoid and Shelf Points')
grid on
