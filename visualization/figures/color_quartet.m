function g = color_quartet(g, num_verts)
    found_any = false; % Flag to track if any patches were found
    
    % Nested function for recursion
    function check_children(parent)
        for i = 1:numel(parent.Children)
            obj = parent.Children(i);
            
            % If the object is a patch, check its vertex count
            if isa(obj, 'matlab.graphics.primitive.Patch')
                verts = obj.Vertices;
                if size(verts, 1) == num_verts
                    obj.FaceColor = [0 1 0]; % Set color to green
                    found_any = true;
                end
            end
            
            % Recursively check children of the current object
            if ~isempty(obj.Children)
                check_children(obj);
            end
        end
    end

    % Start recursion from the main axes object
    check_children(g);

    % Display a message if no matching patches were found
    if ~found_any
        disp("No patches found with " + num2str(num_verts) + " vertices in the axis.");
    end
end
