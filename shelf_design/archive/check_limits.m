clear; close all;
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/add_all_paths.m
run /home/oconnorlabmatlab/Code/libfranka/MATLAB/parameters.m
basefolder = pwd+"/confirmed_positions_3_3/";

file_list = dir(basefolder);
m_arr = {};
for i = 1:numel(file_list)
    filename = file_list(i).folder+"/"+file_list(i).name;
    if ~contains(filename, ".mat")
        continue
    end
    m = load(filename);
    if contains(filename, "success")

        m_arr{end+1} = m;

        % Test if limits respected
        combined = m.combined;
        
        aboveMin = all(all(combined(:,1:7) > jointMin));
        belowMax = all(all(combined(:,1:7) <= jointMax));

        if ~aboveMin || ~belowMax
            disp(num2str(i) + " failed.")
        end

        if ~aboveMin
            difference = min(combined(:,1:7) - jointMin);
            arr = [];
            for j = 1:numel(difference)
                if difference(j) < 0
                    arr(end+1) = difference(j);
                end
            end
            disp(arr);
        end

        if ~belowMax
            difference = max(combined(:,1:7) - jointMax);
            arr = [];
            for j = 1:numel(difference)
                if difference(j) > 0
                    arr(end+1) = difference(j);
                end
            end
            disp(arr);
        end



    elseif contains(filename, "failure")
    else
        disp('Filename does not contain success or failure')
    end
end


