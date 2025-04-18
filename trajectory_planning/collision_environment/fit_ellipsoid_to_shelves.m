function [optimal_radii, optimal_center_Z, Z_list, radius_list, adjusted_pts] = fit_ellipsoid_to_shelves(params)

shelf_pts_map = containers.Map('KeyType','char','ValueType','any');
slot_names = [];
adjusted_pts = [];
slots_for_sphere_dir = params.slots_for_sphere_dir;
valid_list = dir(slots_for_sphere_dir);

for i = 1:numel(valid_list)
    fname = valid_list(i).name;
    if ~contains(fname, ".txt")
        continue
    end

    [~, val] = readSlot(fname);
    xyz = val.O_T_EE(13:15)';

    z = xyz(3) - (params.DIST_FINGERTIP_ABOVE_SHELF + params.SHELF_THICKNESS/2);
    xy = xyz(1:2);
    vec = xy / norm(xy,2);
    xy = xy -vec * params.DIST_FINGERTIP_FROM_INNER_SHELF;
    xyz_new = [xy(1), xy(2), z];

    adjusted_pts = [adjusted_pts;xyz_new];
    slot_names = [slot_names; fname];

    shelf_name = fname(1:2);
    try
        shelf_pts_map(shelf_name) = [shelf_pts_map(shelf_name);xyz_new];
    catch
        shelf_pts_map(shelf_name) = [xyz_new];
    end
end


Z_list = [];
for i =0:12
    shelf_name = sprintf('%02d', i);
    shelf_pts = shelf_pts_map(shelf_name);
    z = shelf_pts(:,3);
    assert(max(z) - min(z) < 0.025);
    Z_list = [Z_list, mean(z)];
end

radius_list = [];
for i = 0:12
    shelf_name = sprintf('%02d', i);
    shelf_pts = shelf_pts_map(shelf_name);
    xy = shelf_pts(:,1:2);
    dists = sqrt(sum(xy.^2, 2));
    assert(max(dists) - min(dists) < 0.05);
    radius_list = [radius_list; min(dists)];
end

[knn, D] = knnsearch(adjusted_pts, adjusted_pts, "K", 2);
nn_D = D(:,2);
assert(all(nn_D >= 0.03), "Some slots are too close to their neighbors.")


% Omit bottom shelf points for ellipsoid, since bottom cylinder controls
% this
adjusted_pts_sans_shelf_0 = adjusted_pts;
z_threshold = mean(Z_list(1:2));  % Average between 0th and 1st shelves
adjusted_pts_sans_shelf_0(adjusted_pts_sans_shelf_0(:,3) < z_threshold, :) = [];

fun = @(x) ellipsoid_cost(x, adjusted_pts_sans_shelf_0);
% Initial guess: [a, b, c, z0] -> axis lengths and center z
x0 = [0.80, 0.92, 0.27];
optimal_x = fminsearch(fun, x0);
optimal_radii = [optimal_x(1), optimal_x(1), optimal_x(2)];
optimal_center_Z = optimal_x(3);


end

function cost = ellipsoid_cost(x, pts)
    a = x(1); b = x(1); c = x(2); z0 = x(3);  % Clamp x and y radius to be the same
    pts_shifted = [pts(:,1), pts(:,2), pts(:,3) - z0];

    norm_vals = (pts_shifted(:,1)/a).^2 + (pts_shifted(:,2)/b).^2 + (pts_shifted(:,3)/c).^2;
    outside = norm_vals > 1;

    volume = 4/3 * pi * a*b*c;

    % Penalize negative radii
    penalty = 0;
    if a<0
        penalty = penalty + -a;
    end
    if b<0
        penalty = penalty + -b;
    end
    if c<0
        penalty = penalty + -c;
    end
    cost = -abs(volume) + -sum(outside) + penalty;

end