function [q, solnInfo] = find_XYZ_q(panda_sc, body_name,XYZ, theta, ik, initialGuess)            
            T_A0 = eye(4);
            Rz = [cos(theta) -sin(theta) 0;
                sin(theta) cos(theta) 0;
                0 0 1];
            T_A0(1:3,1:3) = Rz;
            T_A0(1:3,4) = XYZ';

            T = T_A0;


            attempt_num = 1;
            while attempt_num < 10

                [q,solnInfo] = ik(body_name,T,[1 1 1 1 1 1],initialGuess);

                if  (strcmp(solnInfo.Status, "success")) && (~checkCollision(panda_sc, q))
                    break
                else
                    %                 disp('Collision detected')
                    initialGuess = randomConfiguration(panda_sc);
                    initialGuess(8:9) = 0.01;
                    attempt_num = attempt_num + 1;
                end
            end

end

