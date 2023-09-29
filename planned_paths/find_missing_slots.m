% Calculate poses for missing slots
clear; close all;
addpath("..")
params = CustomParameters();

run list_missing_slots.m


missing_poses_map = containers.Map('KeyType','char','ValueType','any');


%% Calculate poses for each panel that has 3 slot recordings
center_arr = [];
for shelf_name_idx = 1:13
    shelf_name = params.shelf_names(shelf_name_idx);

    for panel_name_idx = 1:3
        panel_name = params.panel_names(panel_name_idx);

        % Get 3 poses from this shelf+panel
        shelf_panel = shelf_name + panel_name;
        disp(shelf_panel)

        % Search for slots within panel that were found
        found_within_panel = [];
        for i = 1:numel(recorded)

            recorded_name = recorded(i);
            if contains(recorded_name, shelf_panel)
                found_within_panel = [found_within_panel; recorded_name];
            end
        end

        found_within_panel = sort(found_within_panel); % Alphebetize, important for indexing later

        slotA = found_within_panel(1);
        slotB = found_within_panel(round(numel(found_within_panel)/2));
        slotC = found_within_panel(end);

        idxA = slotA{1}(4:5);
        idxB = slotB{1}(4:5);
        idxC = slotC{1}(4:5);

        % Read poses
        try
            [~, val_A] = readSlot("/home/oconnorlab/Code/libfranka/MATLAB/planned_paths/manual_slots_valid/"+slotA+".txt");
        catch
            [~, val_A] = readSlot("/home/oconnorlab/Code/libfranka/MATLAB/planned_paths/manual_slots_invalid/"+slotA+".txt");
        end

        [~, val_A] = read_slot_from_valid_or_invalid_dir(slotA);
        [~, val_B] = read_slot_from_valid_or_invalid_dir(slotB);
        [~, val_C] = read_slot_from_valid_or_invalid_dir(slotC);

        % Extract Poses
        T_A = reshape(val_A.O_T_EE, [4,4]);
        T_B = reshape(val_B.O_T_EE, [4,4]);
        T_C = reshape(val_C.O_T_EE, [4,4]);

        T_cell = {};
        T_cell{1} = T_A;
        T_cell{2} = T_B;
        T_cell{3} = T_C;

        idx_list = [str2num(idxA), str2num(idxB), str2num(idxC)];

        [pose_map, center] = interpolate_poses(T_cell, idx_list, shelf_lengths(str2num(shelf_name)+1));  % +1 for shelf indexing from 0
        center_arr = [center_arr;center];

        % Search for slots within panel that were missing
        missing_from_panel = [];
        for i = 1:numel(missing)

            missing_name = missing(i);
            if contains(missing_name, shelf_panel)
                missing_from_panel = [missing_from_panel; missing_name];
            end
        end

        % Create list of missing slot poses
        for missing_idx = 1:numel(missing_from_panel)

            missing_name = missing_from_panel(missing_idx);
            slot_name = missing_name{1}(4:5);
            slot_pose = pose_map(slot_name);
            missing_poses_map(missing_name) = slot_pose;

        end
    end
end


%% Tally up everything
disp(num2str(numel(all_slots)) + " total slots on shelves.")

disp(num2str(numel(recorded)) + " recorded positions.")
disp(num2str(numel(missing)) + " missing positions.")
disp(num2str(missing_poses_map.Count) + " poses in missing_poses_map")


% Show missing vs missing_poses_map
for i = 1:numel(missing)

    not_found = true;

    try
        missing_poses_map(missing(i));
    catch
        disp(missing(i))
    end

end


%
% %% Plot missing
% close all;
% key_list = missing_poses_map.keys;
%
% plotTransforms(se3(missing_poses_map(key_list{1}))); hold on;
%
% for i = 1:numel(key_list)
%     plotTransforms(se3(missing_poses_map(key_list{i})));
% end

%% Save
save("missing_poses.mat", "missing_poses_map", "-mat")

function [q, val_A] = read_slot_from_valid_or_invalid_dir(slot_name)
try
    [q, val_A] = readSlot("/home/oconnorlab/Code/libfranka/MATLAB/planned_paths/manual_slots_valid/"+slot_name+".txt");
catch
    try
        [q, val_A] = readSlot("/home/oconnorlab/Code/libfranka/MATLAB/planned_paths/manual_slots_invalid/"+slot_name+".txt");
    catch
        disp("Could not fine slot ID " + slot_name)
        return
    end
end
end


