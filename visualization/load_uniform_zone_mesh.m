function fv = load_uniform_zone_mesh(params_MATLAB)
    % Ensure that mesh_params used in Python match the relevant parameters in MATLAB.
    
    params_file = params_MATLAB.CustomParametersDir+"/visualization/mesh_params.json";
    
    if isfile(params_file)
        % Read JSON parameters from Python
        fid = fopen(params_file);
        raw = fread(fid, inf);
        fclose(fid);
        json_str = char(raw');
        params_python = jsondecode(json_str);
        
        % Get the property names of the MATLAB CustomParameters object
        matlab_fields = properties(params_MATLAB);
        python_fields = fieldnames(params_python);
        
        mismatch_found = false;

        for i = 1:length(python_fields)
            field_name = python_fields{i};
            
            % Ensure the field exists in params_MATLAB properties
            if any(strcmp(matlab_fields, field_name))
                % Compare the values
                value_matlab = params_MATLAB.(field_name)(:);
                value_python = params_python.(field_name)(:);
                
                % Use an appropriate comparison method for numbers and arrays
                if isnumeric(value_matlab) && isnumeric(value_python)
                    if ~isequal(size(value_matlab), size(value_python)) || any(abs(value_matlab(:) - value_python(:)) > 1e-4)
                        fprintf('Mismatch in parameter: %s\n', field_name);
                        mismatch_found = true;
                    end
                elseif ischar(value_matlab) || isstring(value_matlab)
                    if ~strcmp(string(value_matlab), string(value_python))
                        fprintf('Mismatch in parameter: %s\n', field_name);
                        mismatch_found = true;
                    end
                else
                    fprintf('Warning: Unable to compare parameter %s (type mismatch).\n', field_name);
                    mismatch_found = true;
                end
            else
                fprintf('Warning: Parameter %s is missing in MATLAB params.\n', field_name);
                mismatch_found = true;
            end
        end
        
        % If any mismatch is found, abort
        if mismatch_found
            error('Mesh parameters do not match! STL import aborted.');
        else
            % Load STL if parameters match
            fv = stlread('uniform_zone_mesh.stl');
%             faces = fv.ConnectivityList;
%             vertices = fv.Points;
%             patch('Faces', faces, 'Vertices', vertices, 'FaceColor', 'cyan', "EdgeColor",'none','FaceAlpha',0.25);
%             axis equal;
%             uniform_zone_mesh = struct('Faces', faces, 'Vertices', vertices);
        end
    else
        error('Parameter file missing! Cannot validate STL.');
    end
end
