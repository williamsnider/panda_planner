 filename = "ballpark/ballpark_shelf_1_and_th_0p79_home_to_out_5%.csv";
show(panda_sc, params.q_home)
plotCSV(panda_sc, filename, env, params)

% Plot points too
% Shelf inputs
th = linspace(0, 2*pi,30);

% Points
pts = [];
for Z_num = 1:numel(params.Z_list)

    % Get Z-height and radius of shelf
    Z = params.Z_list(Z_num); 
    radius = params.radius_list(Z_num);

    % Calculate x,y,z
    x = radius*cos(th);
    y = radius*sin(th);
    z = repelem([Z], numel(th));
    pts = [pts;[x',y',z']];
end



% Plot Points
plot3(pts(:,1), pts(:,2), pts(:,3), "k*")
hold on