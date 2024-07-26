function TA = calc_TA(XYZ, theta, W_SHIFT)

            % Position pointing vertically
            T_A0 = eye(4);
            Rz = [cos(theta) -sin(theta) 0;
                sin(theta) cos(theta) 0;
                0 0 1];
            T_A0(1:3,1:3) = Rz;
            T_A0(1:3,4) = XYZ';
            TA = T_A0;

end