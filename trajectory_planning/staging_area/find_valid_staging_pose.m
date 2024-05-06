function [success,q_e, q_s] = find_valid_staging_pose(panda_sc,ik, T_extreme,travel_dist, J7_cutoff)

% Initialize returned variables
success = false;
q_e = zeros(9);
q_s = zeros(9);

% Generate q_extreme
if stinitialGuess = randomConfiguration(panda_sc);
initialGuess(8:9) = 0.01;
[q_extreme,solnInfo] = ik('panda_hand_tcp',T_extreme,[1 1 1 1 1 1],initialGuess);
rcmp(solnInfo.Status, "best available")
    return
end

% Test that J7 rotations are valid
e = q_extreme;
all_e = repmat(e,4,1);
all_e(:,7) = all_e(1,7):pi/2:all_e(1,7)+3*pi/2;
for i = 1:4
    if all_e(i,7) > 2.8
        all_e(i,7) = all_e(i,7) - 2*pi;
    end
end
if any(abs(all_e(:,7)) > 2.89-J7_cutoff)
    return
end

% Generate q_staging 
T_staging = T_extreme;
T_staging(1,4) = T_staging(1,4) + travel_dist;
[q_staging,solnInfo] = ik('panda_hand_tcp',T_staging,[1 1 1 1 1 1],q_extreme);
if strcmp(solnInfo.Status, "best available")
    return
end

% Test that J7 rotations are valid
s = q_staging;
all_s = repmat(s,4,1);
all_s(:,7) = all_s(1,7):pi/2:all_s(1,7)+3*pi/2;
for i = 1:4
    if all_s(i,7) > 2.8
        all_s(i,7) = all_s(i,7) - 2*pi;
    end
end
if any(abs(all_s(:,7)) > 2.89-J7_cutoff)
    return
end

% Check validity
for i = 1:4
        
        % Check self collisions
        if is_robot_in_self_collision_ignore_pairs(panda_sc, all_s(i,:))
            return
        elseif is_robot_in_self_collision_ignore_pairs(panda_sc, all_e(i,:))
            return
        end

        % Check travel distance of joints
        MAX_DIST = 3.0;
        dists = abs(all_s(i,:) - all_e(i,:));
        if any(dists>MAX_DIST)
            disp("Rejected for travel distance.")
            return
        end
end

% Assign output variables if not returned yet
success=true;
q_e = q_extreme;
q_s = q_staging;


end

