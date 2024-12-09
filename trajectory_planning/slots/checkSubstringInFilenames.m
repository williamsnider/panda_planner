function exists = checkSubstringInFilenames(directoryPath, substring)
    % Check if a substring is present in any filenames within a directory
    %
    % Parameters:
    %   directoryPath: Path to the directory
    %   substring: The substring to search for
    %
    % Returns:
    %   exists: Logical true if substring is found in any filename, false otherwise
    
    if nargin < 2
        error('You must provide both the directory path and the substring to search for.');
    end

    % Get list of all files in the directory
    files = dir(directoryPath);
    
    % Filter out directories
    filenames = {files(~[files.isdir]).name};
    
    % Check if any filename contains the substring
    exists = any(contains(filenames, substring));
end
