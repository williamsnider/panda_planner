% Clear all variables and reload from quartet common
run quartet_common.m

%% Find optimal XYZ position
XYZ_list = [];
for X = -0.65:-0.02:-0.80
    for Y = 0.0:0.02:0.20
        for Z = (12.5:1:25.5)*0.0254
            XYZ = [X,Y,Z];
            XYZ_list = [XYZ_list;XYZ];
        end
    end
end

%% Explore positions for horizontal

parfor XYZ_num = 1:size(XYZ_list,1)

%     disp(strcat(num2str(XYZ_num), " of ", num2str(size(XYZ_list,1))))

    warning('off', 'all');

    XYZ = XYZ_list(XYZ_num,:);

    qA_success = zeros(numel(body_names), numel(theta_list));
    qW_success = zeros(numel(body_names), numel(theta_list));
    for body_num = 1:numel(body_names)
        body_name = body_names{body_num};
        for theta_num = 1:numel(theta_list)
            theta = theta_list(theta_num);

            % Calculate correct TA and TW
            [TA, TW] = calc_TA_TW(XYZ, theta, W_SHIFT);

            % Vertical
            initialGuess = randomConfiguration(panda_sc_A);
            [qA, solnInfoA] = find_q_from_T(panda_sc_A,body_name, TA, ik_A, initialGuess);

            % Horizontal
            initialGuess = randomConfiguration(panda_sc_W);
            [qW, solnInfoW] = find_q_from_T(panda_sc_W,body_name,TW, ik_W, initialGuess);




            % Record
            if (~is_robot_in_self_collision_ignore_pairs(panda_sc_A, qA)) && (strcmp(solnInfoA.Status, "success"))
                qA_success(body_num, theta_num) = 1;
            end

            % Record
            if (~is_robot_in_self_collision_ignore_pairs(panda_sc_W, qW)) && (strcmp(solnInfoW.Status, "success"))
                qW_success(body_num, theta_num) = 1;
            end

        end
    end

    Anum_invalid = numel(qA_success)-sum(sum(qA_success));
    Wnum_invalid = numel(qW_success)-sum(sum(qW_success));

    if (Wnum_invalid ==0) &&  (Anum_invalid == 0)
        result = strcat("W: ", num2str(XYZ), "      : ", num2str(Wnum_invalid),"\n","A: ", num2str(XYZ), "      : ", num2str(Anum_invalid));
        disp(result)
%         result = strcat("A: ", num2str(XYZ), "      : ", num2str(Anum_invalid));
%         disp(result)
%         disp("*****")
    end

end

