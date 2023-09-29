
shelf_names = params.shelf_names;
panel_names = params.panel_names;
shelf_lengths = params.shelf_lengths;
omit_names = ["02A50", "02B50", "04A52", "04B52", "05A52", "05B52", "06A52", "06B52", "08A50", "08B50", "10A44", "10B44", "12A32", "12B32"];

% Create container containing shelf names and shelf lengths
shelf_num_slots = containers.Map("KeyType","char","ValueType","any");
for idx = 1:numel(shelf_names)
    shelf_num_slots(char(shelf_names(idx))) = shelf_lengths(idx);
end

% Create list of all
all_slots = [];
for shelf_idx = 1:numel(shelf_names)
    shelf_name = shelf_names(shelf_idx);
    for panel_idx = 1:numel(panel_names)
        panel_name = panel_names(panel_idx);
        for slot_num = 0:2:shelf_lengths(shelf_idx)
            slot_name = shelf_name+panel_name+sprintf( '%02d', slot_num );

            % Omit specific slots
            to_omit = false;
            for omit_idx = 1:numel(omit_names)
                omit_name = omit_names(omit_idx);
                if slot_name == omit_name
                    to_omit=true;
                    break
                end
            end

            if to_omit == false
                all_slots = [all_slots;slot_name];
            end
            
        end
    end
end



% Create list of recorded positions (valid+invalid)
valid = [];
file_list = dir("manual_slots_valid");
for slot_idx = 1:numel(all_slots)
    slot_name = all_slots(slot_idx);

    found_file = false;
    for file_idx = 1:numel(file_list)
        fname = file_list(file_idx).name;
        if contains(fname, slot_name)
            found_file = true;
            break
        end
    end

    if found_file == true
        valid = [valid;slot_name];
    end
end

invalid = [];
file_list = dir("manual_slots_invalid");
for slot_idx = 1:numel(all_slots)
    slot_name = all_slots(slot_idx);

    found_file = false;
    for file_idx = 1:numel(file_list)
        fname = file_list(file_idx).name;
        if contains(fname, slot_name)
            found_file = true;
            break
        end
    end

    if found_file == true

        % Only add if not in valid
        already_in_valid = false;
        for valid_idx = 1:numel(valid)
            valid_name = valid(valid_idx);

            if valid_name == slot_name
                already_in_valid = true;
                break
            end

        end

        if already_in_valid == false
            invalid = [invalid;slot_name];
        end

    end
end

% Combine invalid and valid
recorded = valid;
recorded(end+1:end + numel(invalid),:) = invalid;




% Create not_recorded slots list
not_recorded = [];
for slot_idx = 1:numel(all_slots)
    slot_name = all_slots(slot_idx);

    found_match = false;
    for recorded_idx = 1:numel(recorded)
        recorded_name = recorded(recorded_idx);
        if slot_name == recorded_name
            found_match = true;
            break
        end
    end

    if found_match == false
        not_recorded = [not_recorded;slot_name];
    end
end

% Create missing list (not recorded + invalid)
missing = not_recorded;
missing(end+1:end + numel(invalid),:) = invalid;


% Alphabetize/sort
all_slots = sort(all_slots);
valid = sort(valid);
invalid = sort(invalid);
recorded = sort(recorded);
not_recorded = sort(not_recorded);
missing = sort(missing);

% Check everything is adding up
assert(size(all_slots,1)==size(recorded,1)+size(not_recorded,1));
assert(size(all_slots,1) == size(missing,1) + size(valid,1));

