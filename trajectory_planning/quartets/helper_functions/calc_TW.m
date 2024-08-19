function TW = calc_TW(XYZ, theta)

            % Position pointing vertically
            T_W0 = eye(4);
            Rz = [cos(theta) -sin(theta) 0;
                sin(theta) cos(theta) 0;
                0 0 1];
            T_W0(1:3,1:3) = Rz;
            T_W0(1:3,4) = XYZ';
            TW = T_W0;

end