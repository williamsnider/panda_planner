function [TA, TW] = calc_TA_TW(XYZ, theta, W_SHIFT)

            % Position pointing vertically
            T_A0 = eye(4);
            Rz = [cos(theta) -sin(theta) 0;
                sin(theta) cos(theta) 0;
                0 0 1];
            T_A0(1:3,1:3) = Rz;
            T_A0(1:3,4) = XYZ';
            TA = T_A0;

            % Position pointing horizontally
            T_W0 = eye(4);
            Ry = [cos(-pi/2) 0 sin(-pi/2);
                0 1 0;
                -sin(-pi/2) 0 cos(-pi/2)];
            T_W0(1:3,1:3) = Ry*Rz;
            T_W0(1:3,4) = XYZ' + W_SHIFT';
            TW = T_W0;
end

