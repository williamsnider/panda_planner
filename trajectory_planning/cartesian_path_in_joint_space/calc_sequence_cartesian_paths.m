function [all_paths, all_valid, all_wpts] = calc_sequence_cartesian_paths(robot_ec, robot_sc,env, vScale, jointLimits, poseArray,q_start, params)

% Check inputs
assert(size(poseArray,1)==4);
assert(size(poseArray,2)==4);

num_poses = size(poseArray, 3);
num_paths = num_poses-1;

% Calculate resulting paths
all_paths={};
all_valid = true;
q_initial = q_start;
all_wpts = [];
for i = 1:num_paths
    [q, wpts,valid] = calc_cartesian_path(robot_ec, robot_sc,env, vScale,jointLimits, poseArray(:,:,i),poseArray(:,:,i+1), q_initial,params);
    if valid == false
        all_valid = false;
        break
    else
        all_paths{end+1} = q;
        q_initial = q(end,:); % Next path starts at end of current
        all_wpts = [all_wpts; wpts];
    end

end


end
