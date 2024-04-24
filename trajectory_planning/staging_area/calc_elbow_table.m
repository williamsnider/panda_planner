function [elbow_table, q_staging] = calc_elbow_table(q_extreme,robot_sc, TRAVEL_DIST,ik, SAVE_DIR, letter)


%% Calculate steps between staging and extreme
T_e = getTransform(robot_sc, q_extreme, 'panda_hand_tcp');

x_interval = T_e(1,4):0.0001:T_e(1,4)+TRAVEL_DIST;

num_x = size(x_interval,2);
q_arr = zeros(num_x, 9);

q_arr(1,:) = q_extreme;
for i = 2:num_x
    disp(i)
    % Calculate intermediate pose
    T_x = T_e;
    T_x(1,4) = x_interval(i);

    % Do inverse kinematics, using previous as seed
    initialGuess = q_arr(i-1,:);
    [q_x,solnInfo] = ik('panda_hand_tcp',T_x,[1 1 1 1 1 1],initialGuess);

    if strcmp(solnInfo.Status, "best available")
        disp("ik failed.")
        return
    end

    q_arr(i,:) = q_x;

end

% Flip to go from staging to extreme
x_interval = flip(x_interval);
q_arr = flip(q_arr,1);
q_staging = q_arr(1,:);

% Construct elbow table
elbow_table = [x_interval', q_arr(:,3), sign(q_arr(:,4))];

% Checks
assert(elbow_table(1,1) > elbow_table(end,1));
assert(elbow_table(1,2) == q_staging(3))
assert(elbow_table(end,2) == q_extreme(3));

% Write to CSV
fname = SAVE_DIR+"staging_"+letter+"_elbow_table.csv";
writematrix(elbow_table, fname);

end

