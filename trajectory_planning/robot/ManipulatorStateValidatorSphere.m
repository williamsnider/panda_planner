classdef ManipulatorStateValidatorSphere < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.

    %ManipulatorStateValidator Validator for a joint configuration of a rigid body tree
    %   The validator takes an instance of a robotics.manip.internal.ManipStateSpace
    %   and validates the configuration against the environment.

    %   Copyright 2020-2022 The MathWorks, Inc.

    %#codegen

    properties
        %StateSpace The input ManipStateSpace
        StateSpace

        %Environment The collision objects in the environment
        Environment

        %ValidationDistance The resolution at which motion is validated
        ValidationDistance

        %Robot The underlying Robot of the state space
        Robot_sc

        %Robot The underlying Robot of the state space (environmental
        %collision)
        Robot_ec

        %IgnoreSelfCollision Flag to ignore self collisions during state
        %validation
        IgnoreSelfCollision = false

        %SkippedSelfCollisionType
        SkippedSelfCollisionType = robotics.manip.internal.SkippedSelfCollisionType.PARENT;  % William changed from ADJACENT to PARENT

        % Sphere-checking values
        params
        sphere_origin
        sphere_radius
        radius_offset
        collision_mesh_points
        ismotionvalidcount=0

    end

    methods
        function obj = ManipulatorStateValidatorSphere(ss, env, validationDistance,radius_offset, params)
            %ManipulatorStateValidator Constructor

            %There is no need to check for validity of the input state space ss
            %as the API is internal.
            obj.StateSpace = ss;
            obj.Environment = env;
            obj.ValidationDistance = validationDistance;
            obj.Robot_sc = ss.Robot_sc;
            obj.Robot_ec = ss.Robot_ec;
            obj.params = params;

            % Sphere checking parameters
            obj.sphere_origin = obj.params.sphere_origin;
            obj.sphere_radius = obj.params.sphere_radius;
            obj.radius_offset = radius_offset;
            data = load("collision_mesh_points.mat","collision_mesh_points");
            obj.collision_mesh_points = data.collision_mesh_points;
        end

        function valid = isStateValid(obj, state)


            %isStateValid Validate the input joint state
            %   The method checks if the input state of the rigid body tree is
            %   in collision

            %A state is valid if the robot is not in collision. Calling
            %"checkWorldCollision" and "checkSelfCollision" with "isExhaustive" as
            %false to early exit at the first encounter of collision.
            numStates = size(state, 1);
            valid = true(numStates, 1);

            % Check if within sphere
            for i = 1:numStates
                % Check self collision with robot_sc and environment
                % collision with robot_ec
                tTree = obj.Robot_sc.TreeInternal.forwardKinematics(state(i,:));

                % Check that collision mesh within sphere
                mesh_names = fieldnames(obj.collision_mesh_points);
                robot_base_name = obj.Robot_ec.BaseName;
                robot_body_names = obj.Robot_ec.BodyNames;
                all_xyz = [];
                for j = 1:numel(mesh_names)
                    name = mesh_names(j);
                    name = name{1}; % Cell array to string
                    body_name = ['panda_', name];
                    xyz = obj.collision_mesh_points.(name);
                    xyzw = [xyz, ones(size(xyz,1),1)];  % homogenous coordinates;

                    foundMatch = false;

                    % Test if base
                    if strcmp(body_name, robot_base_name)
                        foundMatch = true;
                        T = eye(4);  % base frame, so identity T matrix
                    end

                    % Test if other body
                    for k = 1:numel(robot_body_names)
                        if strcmp(robot_body_names{k}, body_name)
                            T = tTree{k}';
                            foundMatch = true;
                            break
                        end
                    end

                    assert(foundMatch, 'No matching body in rigidbodytree found.')
                    xyz_T = xyzw*T;
                    xyz_base = xyz_T(:, 1:end-1); % drop 1's column of homogenous coordinates
                    all_xyz = [all_xyz;xyz_base];

                    % Check if within sphere AND below / above cutoffs
                    % (otherwise sphere could go through floor -- bad)
                    xyz_shift = xyz_base - obj.sphere_origin;
                    dists = sqrt(sum(xyz_shift.^2,2));
                    within_sphere = dists < (obj.sphere_radius-obj.radius_offset);

                    % Check if within bottom cylinder
                    vc1=[0,0,-obj.params.cylinder_height/2 + obj.params.sphere_cutoff_bottom]';
                    vc2=[0,0, obj.params.cylinder_height/2+ obj.params.sphere_cutoff_bottom]';
                    d=2*obj.params.bottom_cylinder_radius;
                    within_bottom=CheckPointCylinder(vc1,vc2,d,xyz_base')';

                    % Check if within top cylinder
                    vc1=[0,0,-obj.params.cylinder_height/2 + obj.params.sphere_cutoff_top]';
                    vc2=[0,0, obj.params.cylinder_height/2+ obj.params.sphere_cutoff_top]';
                    d=2*obj.params.top_cylinder_radius;
                    within_top=CheckPointCylinder(vc1,vc2,d,xyz_base')';

                    % Check if within staging box
                    within_staging_box = CheckPointBox(xyz_base, obj.params.staging_box_width, obj.params.staging_box_height, obj.params.staging_box_length, obj.params.staging_box_center);



                    all_valid = (within_sphere|within_bottom|within_top|within_staging_box);

                    if any(~all_valid)
                        valid(i) = false;
                        %                         disp("SV - Sphere collision")
                        %                             show(obj.Robot_sc, state); hold on;plotJointMotion(obj.Robot_sc, state, obj.Environment, obj.params)
                        %                         plotJointMotion(obj.Robot_sc, state, obj.Environment, obj.params)
                        %                         plot3(xyz_base(:,1),xyz_base(:,2),xyz_base(:,3),"*r")
                        return;
                        %                         disp('Violated')
                    end
                end
            end




            % Check if within
            for i = 1:numStates

                % Check for environment collision
                is_colliding_with_world = checkCollision(obj.Robot_ec, state, obj.Environment, "IgnoreSelfCollision","on");
                if is_colliding_with_world
                    disp("SV - World collision")
                    valid(i) = false;
                    return
                end

                % Check for self collisions (ignoring certain adjacent
                % pairs often incorrectly in self-collision)

                %                 isColliding = (checkWorldCollision(obj.Robot_ec.TreeInternal, tTree, baseTform, obj.Environment, false)) || ...
                %                     (~obj.IgnoreSelfCollision && ...
                %                      checkSelfCollision(obj.Robot_sc.TreeInternal, tTree, baseTform, false, ...
                %                                         obj.SkippedSelfCollisionType));
                is_colliding_with_self = is_robot_in_self_collision_ignore_pairs(obj.Robot_sc, state);

                if is_colliding_with_self 
                   disp("SV - self collision")
                   valid(i) = false;
                    return
                end


            end

        end





        % Plot to visualize
        % plotJointMotion(obj.Robot_sc, state(i,:), obj.Environment, obj.params)


        % %                 Plot to visualize
        %                 show(obj.Robot_ec, state(i,:), 'Visuals', 'off', 'Collisions','on')
        %                 hold on;
        %                 plot3(all_xyz(:,1), all_xyz(:,2), all_xyz(:,3), 'r*')
        %
        %                 [x,y,z] = sphere;
        %                 x = x*obj.sphere_radius + obj.sphere_origin(1);
        %                 y = y*obj.sphere_radius + obj.sphere_origin(2);
        %                 z = z*obj.sphere_radius + obj.sphere_origin(3);
        %                 surf(x,y,z, 'EdgeColor','none','FaceAlpha',0.2,'FaceColor',[0.3010, 0.7450, 0.9330])



        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
            %             disp(obj.ismotionvalidcount)
            obj.ismotionvalidcount = obj.ismotionvalidcount+1;
            %isMotionValid Validate the motion between two configurations
            %   The function interpolates between the input configurations and
            %   validates each state in the interpolation.  Note that this
            %   function doesn't output lastValid as in the case of a
            %   nav.StateValidator as this validator will be used for an RRT
            %   based planner

            %Assume that there is no collision and the motion is valid
            isValid = true;
            lastValid = nan(size(state1));

            %Find the ratios at which the interpolated states are obtained, and
            %check each of the interpolated states for validity.
            dist = obj.StateSpace.distance(state1, state2);
            if(dist == 0)
                isValid = obj.isStateValid(state1);
                if(isValid)
                    lastValid = state1;
                end
                return;
            end
            distances = 0:obj.ValidationDistance:dist;

            % Check for validity of the states between state1 and state2
            % (inclusive)
            ratios = [distances/dist, 1];

            % We cannot assume that linearly interpolating between state1
            % and state2 is sufficient for testing the validity of each state. Because we have kinematic
            % constraints,the actual trajectory calculated from the path
            % will be S-shaped for each joint.Therefore we must use the
            % actual trajectory as the interpolation.
            obj.params.vScale = 0.7;
            obj.params.vMaxAll = obj.params.vMaxAllAbsolute*obj.params.vScale;
            traj = joint_path_to_traj([state1;state2], obj.params);
            interpStates = traj;
            interpStates = interpStates(1:obj.params.checkSteps:end,:);

            % linearly-spaced interpolation -  use both linear and
            % kinematically constrained at 70% to cover all bases (linear
            % can be thought of as having a very low vScale).
            interpStates = [interpStates; obj.StateSpace.interpolate(state1, state2, ratios)];

            % Shuffle states to fail faster (sample entire trajectory
            % sooner)
            interpStatesShuffle = interpStates(randperm(size(interpStates,1)),:);

            for i = 1:size(interpStates, 1)
                q = interpStatesShuffle(i,:);
                if(~obj.isStateValid(q))
                    isValid = false;
                    return;
                end
                lastValid = interpStates(i,:);
            end
        end
    end
end
