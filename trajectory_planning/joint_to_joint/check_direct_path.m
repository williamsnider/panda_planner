function [planned_path, IsDirectValid] = check_direct_path(sv, start, goal)
%CHECK_DIRECT_PATH Summary of this function goes here
%   Detailed explanation goes here
planned_path = [start;goal];
IsDirectValid = sv.isMotionValid(start, goal);

end

