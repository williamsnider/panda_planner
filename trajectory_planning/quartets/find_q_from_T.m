function [q, solnInfo] = find_q_from_T(panda_sc, body_name,T, ik, initialGuess)            
            %% Performs inverse kinematics and checks that (1) a q can be found at that T and (2) the robot is not in self collision at this q.
            attempt_num = 1;
            while attempt_num < 10

                [q,solnInfo] = ik(body_name,T,[1 1 1 1 1 1],initialGuess);

                % Exit if failed
                if ~strcmp(solnInfo.Status, "success")
                    break
                end

                if  (strcmp(solnInfo.Status, "success")) && (~is_robot_in_self_collision_ignore_pairs(panda_sc, q))
                    break
                
                else
                    %                 disp('Collision detected')
                    initialGuess = randomConfiguration(panda_sc);
                    initialGuess(8:9) = 0.01;
                    attempt_num = attempt_num + 1;
                end
            end

end

