% Code to find the best configurations different assembly types


%% Load variables
clear; close all;
addpath("../..")
params = CustomParameters();
warning('off', 'all');

% Set random stream for reproducibility
stream = RandStream('mt19937ar', 'Seed', 123); 
RandStream.setGlobalStream(stream);

%% Load common assembly parameters
c = assembly_common("radial", 4, params);

body_names = c.body_names;
panda_sc = c.panda_sc;
panda_ec = c.panda_ec;
triObj = c.triObj;
cylinderRadius = c.cylinderRadius;

%% Ik

ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;

%% Do inverse kinematics for shape positions

XYZ = [-0.60, 0.0, 20*0.0254];
initialGuess = randomConfiguration(panda_sc);

for body_num = 1:numel(body_names)
    body_name = body_names{body_num};
    for theta = -pi/4:pi/2:5*pi/4
        
        attempt_num = 1;
        while attempt_num < 10
            [q, solnInfo] = find_XYZ_q(panda_sc,body_name, XYZ, theta, ik);
            
            % Exit loop if ik successful and not in self collision
            if (strcmp(solnInfo.Status, "success") && ~checkCollision(panda_sc, q))
                break
            % Exit loop if ik unsuccessful (no point retrying)
            elseif ~(strcmp(solnInfo.Status, "success"))
                break
            % Repeat loop if ik successful but in self collision
            end

            attempt_num = attempt_num + 1;
        end

        % Check that ik was successful
        if ~strcmp(solnInfo.Status, "success")
            disp("IK failed")
        end

        % Check that not in collision
        if checkCollision(panda_sc, q)
            disp("Self collision")
        end

        plot_assembly(panda_sc, q, triObj, cylinderRadius, XYZ)
        input("")



%         ori_base = eye(3);
%         ori_A0 = ori_base;
%         
%         T_A0 = eye(4);
%         Rz = [cos(theta) -sin(theta) 0; 
%               sin(theta) cos(theta) 0; 
%               0 0 1];
%         T_A0(1:3,1:3) = Rz;
%         T_A0(1:3,4) = XYZ';
%         
%         T = T_A0;
%         ik = inverseKinematics('RigidBodyTree',panda_sc);
%         ik.SolverParameters.MaxIterations = 1000;
%         initialGuess(8:9) = 0.01;
%         [q,solnInfo] = ik(body_name,T,[1 1 1 1 1 1],initialGuess);
%         
%         % Check that ik was successful
%         if ~strcmp(solnInfo.Status, "success")
%             disp("IK failed")
%         end
%         
%         % Display distance between positions
%         dist = sum(abs(initialGuess-q));
%         disp(dist);
% 
%         initialGuess = q;
%         
%         %% Show result
%         plot_assembly(panda_sc, q, triObj, cylinderRadius, ori_base, XYZ)
%         
%         disp(strcat(body_name, "    ",num2str(theta)));
%         input("");
    end
end


