function found = check_if_file_in_dir(substring, directory)

% Get a list of all files and folders in the directory
files = dir(directory);

% Loop through the files to check for the substring
found = false;
for k = 1:length(files)
    if contains(files(k).name, substring)
        found = true;
        break; % Stop the loop if we find the substring
    end
end

end
