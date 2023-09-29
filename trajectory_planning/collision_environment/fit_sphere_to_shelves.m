function [optimal_radius, optimal_sphere_origin_Z, Z_list, radius_list, adjusted_pts] = fit_sphere_to_shelves(params)

% Fit sphere to shelves, which is used by the manipulatorRRTSphere to
% ensure that all points of the robot are WITHIN a sphere (as opposed to
% individually checking collisions with all shelves and objects, which
% takes too long).

% Group points into shelves
shelf_pts_map = containers.Map('KeyType','char','ValueType','any');

% Points - read valid slots
slot_names = [];
adjusted_pts = [];
slots_for_sphere_dir = params.slots_for_sphere_dir;
valid_list = dir(slots_for_sphere_dir);
for i = 1:numel(valid_list)
    fname = valid_list(i).name;

    % Skip "." and ".."
    if ~contains(fname, ".txt")
        continue
    end

    % Read slot, save O_T_EE
    [~, val] = readSlot(fname);
    xyz = val.O_T_EE(13:15)';

    % Shift Z down to center of shelf below
    z = xyz(3) - (params.DIST_FINGERTIP_ABOVE_SHELF + params.SHELF_THICKNESS/2);

    % Shift XYZ toward Z-axis (since slot is slightly inset)
    xy = xyz(1:2);
    vec = xy / norm(xy,2);
    xy = xy -vec * params.DIST_FINGERTIP_FROM_INNER_SHELF;

    xyz_new = [xy(1), xy(2), z];

    adjusted_pts = [adjusted_pts;xyz_new];
    slot_names = [slot_names; fname];

    % Place slot into group
    shelf_name = fname(1:2);
    try
        shelf_pts_map(shelf_name) = [shelf_pts_map(shelf_name);xyz_new];
    catch
        shelf_pts_map(shelf_name) = [xyz_new];
    end
end

% Find optimal sphere
fun = @(x)cost_function(x,adjusted_pts);
optimal_x = fminsearch(fun, [0.5,0.5]);
optimal_radius = optimal_x(1);
optimal_sphere_origin_Z = optimal_x(2);

% Get Z-list (average Z of each shelf)
Z_list = [];
for i =0:12
    shelf_name = sprintf( '%02d', i );
    shelf_pts = shelf_pts_map(shelf_name);
    z= shelf_pts(:,3);
    z_mean = mean(z);
    z_max = max(z);
    z_min = min(z);

    % Check all on same shelf (have similar Z-values)
    assert(z_max - z_min < 0.025, "Slots on the same shelf differ in Z-height by a significant margin. The outlier slot is probably from the wrong shelf.")  % Different shelfs should be 0.1m apart

    Z_list = [Z_list, z_mean];
end


% Get radius list (average radius of each shelf)
radius_list = [];
for i = 0:12
    
    shelf_name = sprintf( '%02d', i );
    shelf_pts = shelf_pts_map(shelf_name);
    xy = shelf_pts(:,1:2);
    dists = sqrt(sum(xy.^2, 2));

    % Check all slots on same shelf (have similar radii)
    assert(max(dists) - min(dists) < 0.05, "Slots on the same shelf differ in their radius by a significant margin. The outlier slot is probably from the wrong shelf.")  

    radius_list = [radius_list; min(dists)];
end

% Check that points are non overlapping (two recordings not mistakenly at
% same slot)
[knn, D] = knnsearch(adjusted_pts, adjusted_pts, "K", 2);
nn_idx = knn(:,2); % Second neighbor = nearest excluding self
nn_D = D(:,2);

% Plot pairs that are too close
THRESHOLD = 0.03;
mask = nn_D < THRESHOLD;
idx1 = find(mask);
assert(numel(idx1)==0, "Some slots are too close to their neighbors.")

% % Plot Points
% plot3(adjusted_pts(:,1), adjusted_pts(:,2), adjusted_pts(:,3), "k*"); hold on
% for j = 1:numel(idx1)
%     idx = idx1(j);
%     plot3(adjusted_pts(idx,1), adjusted_pts(idx,2), adjusted_pts(idx,3), "ro")
%     text(adjusted_pts(idx,1), adjusted_pts(idx,2), adjusted_pts(idx,3)+0.05, slot_names(idx,:))
%     disp(slot_names(idx,:))
% 
% end

% 
% % Plot fitted sphere
% [X,Y,Z] = sphere(25);
% X = X*optimal_radius ;
% Y = Y*optimal_radius ;
% Z = Z*optimal_radius + optimal_sphere_origin_Z;
% surf(X,Y,Z)
% alpha(0.5)
% axis("equal")
end

function cost = cost_function(x, pts)
    
    sphere_radius = x(1);
    sphere_origin_Z = x(2);

    %% Count number of points inside sphere 
    
    % Shift points to origin of sphere
    pts_shift = pts;
    pts_shift(:,3) = pts_shift(:,3) - sphere_origin_Z;
    
    % Calclute distance to origin (center of sphere)
    pts_dist = vecnorm(pts_shift, 2, 2);

    % Count number of points with a distance smaller than radius (i.e.
    % within the sphere)
    count = sum(pts_dist < sphere_radius);

    %% Cost value
    cost = count - sphere_radius;

end

