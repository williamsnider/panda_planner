% For all shelf heights, find the maximum radius that is within reach
% of the robot (correct position, correct orientation, and the robot can
% pick/place the shape with a valid cartesian path.
clear; close all;
run /home/oconnorlab/code/libfranka/MATLAB/add_all_paths.m
run /home/oconnorlab/code/libfranka/MATLAB/parameters.m

% delete(gcp('nocreate'))
% parpool("Processes", 22)


%% Inputs
ABOVE_HEIGHT = 0.0202; %m; how high the shape is pulled when picking
OUT_DIST = 0.130; %m; how far out the shape is pulled when picking

SHELF_SPACING = 0.1; %m; spacing of shelves (i.e. includes thickness of one shelf)
PEG_DEPTH = 0.0127; %m
HEIGHT_FINGERSLOT_ABOVE_SHELF = 0.0225; %m
SHAPE_MAX_HEIGHT = 0.07; %m
SHAPE_MAX_WIDTH = 0.07; %m
SHELF_RIM_OFFSET = 0.015; %m distance between edge of shelf and slot
POST_LENGTH = 0.019558;
FINGERTIP_SLOT_DIMENSIONS = 0.01; %m
FINGERTIP_SLOT_DISTANCE_FROM_BACK_OF_INTERFACE = 0.002; %m
INTERFACE_HEIGHT = HEIGHT_FINGERSLOT_ABOVE_SHELF+ FINGERTIP_SLOT_DIMENSIONS*sqrt(2)/2 + FINGERTIP_SLOT_DISTANCE_FROM_BACK_OF_INTERFACE; %
INTERFACE_WIDTH = PEG_DEPTH;
INTERFACE_LENGTH =  FINGERTIP_SLOT_DIMENSIONS*sqrt(2) + 2*FINGERTIP_SLOT_DISTANCE_FROM_BACK_OF_INTERFACE; 
RADIUS_SAFETY_MARGIN = 0.075; %m; reduce max radius found above by this amount to ensure robot is not at its joint extremes
SHAPE_SPACING = SHAPE_MAX_WIDTH + 0.005; % Contemplate this more
SHAPE_ROTATION_ABOUT_Z = pi/18; %rad; rotation of shape about z-axis to improve access of robot
DIST_FROM_INTERFACE_OUTER_WIDTH_TO_FINGERSLOT = FINGERTIP_SLOT_DISTANCE_FROM_BACK_OF_INTERFACE + FINGERTIP_SLOT_DIMENSIONS*sqrt(2)/2;
SHAPE_MAX_LENGTH = 0.12 - INTERFACE_LENGTH/2-POST_LENGTH; %m


% Values of z/theta/radii to test
num_theta_samples = 5;
theta_samples = linspace(-3*pi/4, 3*pi/4, num_theta_samples);

z_samples = -4*SHELF_SPACING:SHELF_SPACING:13*SHELF_SPACING;
num_z_samples = numel(z_samples);

radius_samples = 1:-0.005:0.35; % note descending
num_radius_samples = numel(radius_samples);

%% Clear folder where positions are saved
basefolder = pwd+"/saved_shape_positions/";
txt = input("Would you like to delete everything in "+basefolder+"? YES to do so.", "s");
if strcmp(txt,"YES")
    delete(basefolder+"*")
    disp("Deleted all contents.")
else
    disp("Kept all contents.")
end


%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();
env(end) = []; disp('removing monkey grasp region')

%% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree',panda_ec);
ik.SolverParameters.MaxIterations = 2500;
weights = [1 1 1 1 1 1];
ss = manipulatorStateSpace(panda_ec); % for joint limits

%% Preallocate combinations of z/theta/radius for use in parpool
z_theta_radius_combs = [];
for z_idx = 1:num_z_samples
    for theta_idx = 1:num_theta_samples
        for radius_idx = 1:num_radius_samples
            z_theta_radius_combs = [z_theta_radius_combs; z_idx, theta_idx, radius_idx];
        end
    end
end

% %% Test if each combination is valid (i.e. a shape could be picked/placed from there successfully)
% num_combs = size(z_theta_radius_combs,1);
% parfor i_comb = 1:num_combs
% 
%     % Get values of z/theta/radius
%     z_idx = z_theta_radius_combs(i_comb, 1);
%     theta_idx = z_theta_radius_combs(i_comb, 2);
%     radius_idx = z_theta_radius_combs(i_comb, 3);
%     z = z_samples(z_idx);
%     theta = theta_samples(theta_idx);
%     radius = radius_samples(radius_idx);
% 
%     [found_valid_cartesian_path, T, combined, all_paths] = shelf_find_cartesian_path(z,radius,theta, SHAPE_ROTATION_ABOUT_Z, panda_ec, panda_sc, ik, weights, env, ss,ABOVE_HEIGHT, OUT_DIST, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute, jointMin, jointMax );
% 
%     if found_valid_cartesian_path==true
%         filename = basefolder+"combined-z-"+num2str(z_idx)+"_r-"+num2str(radius_idx)+"_theta-" + num2str(theta_idx)+".csv";
%         writematrix(combined,filename);
%         filename = basefolder+"valid-z-"+num2str(z_idx)+"_r-"+num2str(radius_idx)+"_theta-" + num2str(theta_idx)+".csv";
%         xyz = T(1:3,4)';
%         writematrix(xyz,filename);
%     else
%         filename = basefolder+"invalid-z-"+num2str(z_idx)+"_r-"+num2str(radius_idx)+"_theta-" + num2str(theta_idx)+".csv";
%         xyz = T(1:3,4)';
%         writematrix(xyz,filename);
%     end
%     disp("Finished combination "+num2str(i_comb)+" of " + num2str(num_combs))
% end


%% Choose radii for each z
% Read directory
basefolder = pwd+"/saved_shape_positions/";
invalid_xyz = [];
valid_xyz = [];
all_combined = {};
file_list = dir(basefolder);
z_radius_theta = logical(zeros(num_z_samples, num_radius_samples, num_theta_samples));
for i = 1:numel(file_list)
    disp(i)
    filename = file_list(i).folder+"/"+file_list(i).name;

    % Skip "." and ".."
    if ~contains(filename, ".csv")
        continue
    end


    % Add xyz position into array
    if contains(filename,"/invalid")
        xyz=readmatrix(filename);
        invalid_xyz = [invalid_xyz;xyz];
    elseif contains(filename, "/valid")
        xyz=readmatrix(filename);
        valid_xyz = [valid_xyz;xyz];

        % Extract indices by parsing filename
        l = split(filename, "-z-");
        l = l(2);
        r = split(l,"_r-");
        z_idx = str2num(r(1));

        l = split(filename, "_r-");
        l = l(2);
        r = split(l,"_theta-");
        radius_idx = str2num(r(1));

        l = split(filename, "theta-");
        l = l(2);
        r = split(l,".csv");
        theta_idx = str2num(r(1));

        z_radius_theta(z_idx, radius_idx, theta_idx) = 1;

    elseif contains(filename, "/combined-")
        combined=readmatrix(filename);
        all_combined{end+1} = combined;
    else
        error("not implemented");
    end
end

% Find max radius (largest radius that is valid for all 5 thetas)
z_radius_boolean_array = all(z_radius_theta,3);
z_radius_idx_pairs = [];
z_radius_value_pairs = [];
for z_idx = 1:num_z_samples
    radius_idx = find(z_radius_boolean_array(z_idx,:),1 ,"first");
    if numel(radius_idx)~=0
        z_radius_idx_pairs = [z_radius_idx_pairs; [z_idx radius_idx]];

        z_radius_value_pairs = [z_radius_value_pairs; z_samples(z_idx) radius_samples(radius_idx)];
    end
end

% Reduce by safety margin
safety_z_radius_value_pairs = z_radius_value_pairs;
safety_z_radius_value_pairs(:,2) = safety_z_radius_value_pairs(:,2) -  RADIUS_SAFETY_MARGIN;

% Calculate cartesian path for dense sampling of shapes to check


%% Find cartesian paths for given positions

% 2/3 
basefolder = pwd+"/confirmed_positions_2_3/";
safety_combs = [];
factor = 2/3;
for i=1:size(safety_z_radius_value_pairs,1)
    z = safety_z_radius_value_pairs(i,1);
    radius = safety_z_radius_value_pairs(i,2);

    % Determine theta based on needed spacing
    rad_gap = 2*atan(SHAPE_SPACING/2 / radius)*factor;
    sub_theta_samples = -3*pi/4:rad_gap:3*pi/4;
    for j = 1:numel(sub_theta_samples)
        theta = sub_theta_samples(j);
        safety_combs = [safety_combs;z,radius,theta];
    end
end

if isfolder(basefolder)
txt = input("Would you like to delete everything in "+basefolder+"? YES to do so.", "s");
txt="YES";
    if strcmp(txt,"YES")
        delete(basefolder+"*")
        disp("Deleted all contents.")
    else
        disp("Kept all contents.")
    end
else
    mkdir(basefolder)
end

parfor i = 1:size(safety_combs,1)
%     if i < 569
%         continue
%     end
    disp(num2str(i) + " of " + num2str(size(safety_combs,1)))
    z = safety_combs(i,1);
    radius = safety_combs(i,2);
    theta = safety_combs(i,3);
    [found_valid_cartesian_path, T, combined, all_paths] = shelf_find_cartesian_path(z,radius,theta, SHAPE_ROTATION_ABOUT_Z, panda_ec, panda_sc, ik, weights, env, ss,ABOVE_HEIGHT, OUT_DIST, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute, jointMin, jointMax);

    % Save results
    if found_valid_cartesian_path
        filename = basefolder + "success-"+sprintf('%05d', i) + ".mat";
    else
        disp("failed for "+num2str(i))
        filename = basefolder + "failure-"+sprintf('%05d', i) + ".mat";
    end
    m=matfile(filename,'writable',true);
    m.z = z;
    m.radius = radius;
    m.theta = theta;
    m.T=T;
    m.combined=combined;
    m.all_paths = all_paths;
end

% 3/3 
basefolder = pwd+"/confirmed_positions_3_3/";
safety_combs = [];
factor = 3/3;
for i=1:size(safety_z_radius_value_pairs,1)
    z = safety_z_radius_value_pairs(i,1);
    radius = safety_z_radius_value_pairs(i,2);

    % Determine theta based on needed spacing
    rad_gap = 2*atan(SHAPE_SPACING/2 / radius)*factor;
    sub_theta_samples = -3*pi/4:rad_gap:3*pi/4;
    for j = 1:numel(sub_theta_samples)
        theta = sub_theta_samples(j);
        safety_combs = [safety_combs;z,radius,theta];
    end
end

if isfolder(basefolder)
% txt = input("Would you like to delete everything in "+basefolder+"? YES to do so.", "s");
txt="YES";
    if strcmp(txt,"YES")
        delete(basefolder+"*")
        disp("Deleted all contents.")
    else
        disp("Kept all contents.")
    end
else
    mkdir(basefolder)
end

% extra_arr = [431, 459, 462, 498, 530];
% for j = 1:numel(extra_arr)
%     i = extra_arr(j);
parfor i = 1:size(safety_combs,1)
    disp(num2str(i) + " of " + num2str(size(safety_combs,1)))
    z = safety_combs(i,1);
    radius = safety_combs(i,2);
    theta = safety_combs(i,3);
    [found_valid_cartesian_path, T, combined, all_paths] = shelf_find_cartesian_path(z,radius,theta, SHAPE_ROTATION_ABOUT_Z, panda_ec, panda_sc, ik, weights, env, ss,ABOVE_HEIGHT, OUT_DIST, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute, jointMin, jointMax);

    % Save results
    if found_valid_cartesian_path
        filename = basefolder + "success-"+sprintf('%05d', i) + ".mat";
    else
        disp("failed for"+num2str(i))
        filename = basefolder + "failure-"+sprintf('%05d', i) + ".mat";
    end
    m=matfile(filename,'writable',true);
    m.z = z;
    m.radius = radius;
    m.theta = theta;
    m.T=T;
    m.combined=combined;
    m.all_paths = all_paths;
end

%% Test collisions for calculated paths
%2/3
[panda_ec, panda_sc] = loadPandaWithShape();
basefolder = pwd+"/confirmed_positions_2_3/";
factor = 2/3;
new_shape_max_width = 0.070*factor;
disp("Testing 2/3 spacings for " + num2str(numel(dir(basefolder))-2) + " combinations.")
check_no_collision(panda_ec, panda_sc,env, basefolder, INTERFACE_HEIGHT,INTERFACE_WIDTH,INTERFACE_LENGTH,POST_LENGTH, SHAPE_MAX_HEIGHT, new_shape_max_width, SHAPE_MAX_LENGTH, PEG_DEPTH, SHAPE_SPACING, SHELF_RIM_OFFSET, HEIGHT_FINGERSLOT_ABOVE_SHELF)

%3/3
[panda_ec, panda_sc] = loadPandaWithShape();
basefolder = pwd+"/confirmed_positions_3_3/";
factor = 3/3;
new_shape_max_width = 0.070*factor;
disp("Testing 3/3 spacings for " + num2str(numel(dir(basefolder))-2) + " combinations.")
check_no_collision(panda_ec, panda_sc,env, basefolder, INTERFACE_HEIGHT,INTERFACE_WIDTH,INTERFACE_LENGTH,POST_LENGTH, SHAPE_MAX_HEIGHT, new_shape_max_width, SHAPE_MAX_LENGTH, PEG_DEPTH, SHAPE_SPACING, SHELF_RIM_OFFSET, HEIGHT_FINGERSLOT_ABOVE_SHELF)


% basefolder = pwd+"/confirmed_positions_2_3/";
% file_list = dir(basefolder);
% success_xyz = [];
% failure_xyz = [];
% m_arr = {};
% shape_arr = {};
% interface_arr = {};
% post_arr = {};
% shelf_arr = {};
% for i = 1:numel(file_list)
%     filename = file_list(i).folder+"/"+file_list(i).name;
%     if ~contains(filename, ".mat")
%         continue
%     end
% 
%     m = load(filename);
%     if contains(filename, "success")
% 
%         success_xyz = [success_xyz; m.T(1:3,4)'];
%         m_arr{end+1} = m;
% 
% 
% 
%         interface = collisionBox(INTERFACE_HEIGHT, INTERFACE_WIDTH, INTERFACE_LENGTH);
%         T = m.T;
%         interface.Pose = T;
%         interface_arr{end+1} = interface;
% 
%         post = collisionCylinder(INTERFACE_WIDTH/2, POST_LENGTH);
%         T = m.T;
%         vec = T(1:2,3);
%         T(1:2,4) = T(1:2,4) + vec*INTERFACE_LENGTH/2+ vec*POST_LENGTH/2;
%         post.Pose = T;
%         post_arr{end+1} = post;
% 
%         box = collisionBox(SHAPE_MAX_HEIGHT, SHAPE_MAX_WIDTH, SHAPE_MAX_LENGTH);
%         T = m.T;
%         vec = T(1:2,3);
%         T(1:2,4) = T(1:2,4) +  + vec*INTERFACE_LENGTH/2+ vec*POST_LENGTH + vec*SHAPE_MAX_LENGTH/2;
%         box.Pose = T;
%         shape_arr{end+1} = box;
% 
% 
%         % Shelf but without rotation about Z
%         shelf = collisionBox(PEG_DEPTH,SHAPE_SPACING,2*SHELF_RIM_OFFSET+PEG_DEPTH);
%         theta = atan2(m.T(2,4), m.T(1,4));
%         T = eul2tform([0, pi/2,-theta]);
%         T(1:3,4) = m.T(1:3,4);
%         T(3,4) = T(3,4)  - PEG_DEPTH/2 - HEIGHT_FINGERSLOT_ABOVE_SHELF;
%         shelf.Pose = T;
%         shelf_arr{end+1} = shelf;
% 
% 
%     elseif contains(filename, "failure")
%         failure_xyz = [failure_xyz; m.T(1:3,4)'];
%     else
%         disp('Filename does not contain success or failure')
%     end
% 
% end
% 
% i=890;
% disp(num2str(i)+" of " +num2str(numel(m_arr)))
% m = m_arr{i};
% combined = m.combined;
% SAMPLE_DENSITY = 0.01;
% sample_indices = ceil(linspace(1,size(combined,1),ceil(size(combined,1)*0.1)));
% num_samples = numel(sample_indices);
% 
% % Copy collision environment, omitting the shape at this pose
% copy_shape_arr = shape_arr;
% copy_shape_arr(i) = [];  % omit this shape
% copy_interface_arr = interface_arr;
% copy_interface_arr(i) = []; % omit this shape
% copy_post_arr = post_arr;
% copy_post_arr(i) = []; % omit this shape
% sub_arr = [copy_shape_arr, copy_interface_arr,  copy_post_arr, shelf_arr, env];
% 
% % Attach shape
% obj1 = shape_arr{i};
% obj1Body = rigidBody("obj1");
% obj1Joint = rigidBodyJoint("obj1Joint");
% T = getTransform(panda_ec,combined(1,:),"panda_hand_tcp");
% setFixedTransform(obj1Joint,T\obj1.Pose);
% addCollision(obj1Body,obj1,inv(obj1.Pose));
% obj1Body.Joint = obj1Joint;
% addBody(panda_ec,obj1Body,"panda_hand_tcp");


close;
q = combined(1,:)
ax= show(panda_ec, q,"Frames", "off", "Collisions", "on");hold on;
% for i = 1:numel(sub_arr)
%     [~, patchObj] = show(sub_arr{i}, "Parent", ax);
%     patchObj.FaceColor = [1 0.35 1];
%     patchObj.EdgeColor = 'none';
% end
plot3(valid_xyz(:,1), valid_xyz(:,2), valid_xyz(:,3), "r*")
plot3(invalid_xyz(:,1), invalid_xyz(:,2), invalid_xyz(:,3), "b*")
