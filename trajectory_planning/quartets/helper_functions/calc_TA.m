function  TA = calc_TA(XYZ, theta)

            Rz = [cos(theta) -sin(theta) 0;
                sin(theta) cos(theta) 0;
                0 0 1];


            % Position pointing horizontally
            T_A0 = eye(4);
            Ry = [cos(-pi/2) 0 sin(-pi/2);
                0 1 0;
                -sin(-pi/2) 0 cos(-pi/2)];
            T_A0(1:3,1:3) = Ry*Rz;
            T_A0(1:3,4) = XYZ' ;
            TA = T_A0;
end