%% This script is a sanity check that all slots are in the correct location
clear; close all;
addpath("..")
params = CustomParameters();

%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
[env_norm, env_big] = build_collision_environment();


% For each file in "slots" directory, open if it contains "slot_to_above"
% data. Then, read the first row in the csv data. Store in a container that
% maps the two, using the first 14 characters of the filename as the key,
% and the first row of the csv file as the data
% Initialize containers for storing data and poses
dataMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
poseMap = containers.Map('KeyType', 'char', 'ValueType', 'any');


% Then, for each key/value pair, calculate the forward kinematics using the
% getTransform function to determine the pose. Again store the poses in a
% container type structure.
files = dir('slots/*.csv');
for file = files'
    filename = file.name;
    if contains(filename, 'slot_to_above')
        % Extract key and read data
        key = filename(10:14);
        data = readmatrix(fullfile('slots', filename)); % Assuming 6 columns
        q = [data(1,:), 0.01, 0.01];
        dataMap(key) = q;

        % Calculate forward kinematics and store pose
        pose = getTransform(panda_sc, q, "panda_hand_tcp");
        poseMap(key) = pose;
    end
end

% Then, for each key/value pair, compare the xyz position with that of its
% neighbors. Print any erroneous poses (more than 0.05 m away).
keys = poseMap.keys;
panels = ["A", "B", "C"];
count = 0;
for shelf_num = 0:12
    shelf_name = sprintf('%02d', shelf_num);
    for panel_num = 1:3
        panel_name = panels(panel_num);

        for slot_num= 0:2:52
            slot_name = sprintf("%02d", slot_num);

            slot_id_pre = shelf_name + panel_name + sprintf('%02d', slot_num-2);
            slot_id_curr = shelf_name + panel_name + slot_name;
            slot_id_post = shelf_name + panel_name + sprintf('%02d', slot_num+2);

            % Test if all 3 captured
            if ~any(contains(keys, slot_id_pre)) || ~any(contains(keys, slot_id_curr)) ||~any(contains(keys, slot_id_post))
                %             disp("Skipping "+slot_id_curr)
                continue

            end

            count = count+1;

            % Test poses
            pre = poseMap(slot_id_pre);
            curr = poseMap(slot_id_curr);
            post = poseMap(slot_id_post);

            xyz_pre = pre(1:3, 4);
            xyz_curr = curr(1:3,4);
            xyz_post = post(1:3,4);

            dist_pre_curr = norm([xyz_pre-xyz_curr], 2);
            dist_curr_post = norm([xyz_curr-xyz_post],2);

            % Compare
            threshold_min = 0.025;
            threshold_max = 0.065;
            if dist_curr_post<threshold_min || dist_curr_post>threshold_max ||  dist_pre_curr<threshold_min || dist_pre_curr>threshold_max
                disp("******")
                disp("Invalid! for "+ slot_id_curr)
                disp(dist_pre_curr)
                disp(dist_curr_post)
            end

        end

    end
end

% for i = 1:length(keys)
%     key = keys{i};
%     pose1 = poseMap(key);
% 
%     % Compare with neighbors
%     for j = i+1:length(keys)
%         neighbor_key = keys{j};
%         pose2 = poseMap(neighbor_key);
% 
%         % Calculate distance
%         distance = norm(pose1(1:3) - pose2(1:3));
% 
%         % Check if distance is greater than 0.05 m
%         if distance > 0.05
%             fprintf('Erroneous poses detected: %s and %s are %.2f m apart\n', key, neighbor_key, distance);
%         end
%     end
% end
