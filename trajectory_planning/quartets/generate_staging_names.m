function pos_names = generate_staging_names(staging_letters)
%GENERATE_STAGING_NAMES Summary of this function goes here
%   Detailed explanation goes here
pos_names = {};
staging_numbers = ["0","1","2","3"];
staging_alternates = ["a","b"];

for letter_num = 1:numel(staging_letters)
    for number_num = 1:numel(staging_numbers)
        for alternate_num = 1:numel(staging_alternates)

            staging_letter = staging_letters(letter_num);
            staging_number = staging_numbers(number_num);
            staging_alternate = staging_alternates(alternate_num);

            pos_name = strcat(staging_letter, staging_number, staging_alternate);
            pos_names{end+1} = pos_name;
        end
    end
end
end

