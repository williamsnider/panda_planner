classdef manipulatorRRTSphere < matlabshared.planning.internal.BiRRT
%MANIPULATORRRTSphere Plan motion for rigid body tree using bidirectional RRT
%   
%   This class is modified to allow for a ManipulatorStateValidator that
%   enables checking for collisions within a sphere.
% 
% MANIPULATORRRT object is a single-query planner that uses the
%   bidirectional Rapidly-Exploring Random Trees (RRT) algorithm
%   with an optional connect heuristic for increased speed.
%
%   The bidirectional RRT planner creates two trees with root nodes at the
%   specified start and goal configurations. To extend each tree, a random
%   configuration is generated and a step is taken from the nearest node
%   based on the MAXCONNECTIONDISTANCE property. The start and goal trees
%   alternate this extension process until both trees are connected. If the
%   connect heuristic is enabled, the extension process ignores
%   MAXCONNECTIONDISTANCE. Invalid configurations or connections that
%   collide with the environment are not added to the tree.
%
%   manipulatorRRT(RIGIDBODYTREE, ENV) creates a bidirectional RRT planner
%   from a rigidBodyTree object, and a cell-array of collision primitives.
%   The POSE of the collision object is specified in world coordinates.
%
%   manipulatorRRT properties:
%       MaxConnectionDistance    - Maximum length between planned configurations
%       ValidationDistance       - Distance resolution for validating motion between configurations
%       MaxIterations            - Maximum number of random configurations generated
%       EnableConnectHeuristic   - Directly join the start and goal trees
%       WorkspaceGoalRegionBias  - Probability to sample additional goal states within the workspace goal region
%       IgnoreSelfCollision      - Ignore self-collisions during planning
%       SkippedSelfCollisions    - Body pairs skipped for checking self-collisions
%
%   manipulatorRRT methods:
%       plan        - Plan motion from start to goal configuration
%       interpolate - Interpolate states along path
%       shorten     - Trim edges to shorten the planned path
%
%   Example:
%            %Import the robot
%            robot = importrobot("iiwa14.urdf");
%
%            %Create the environment
%            env = {collisionBox(0.5, 0.5, 0.05), collisionSphere(0.3)};
%            env{1}.Pose(3, end) = -0.05;
%            env{2}.Pose(1:3, end) = [0.1, 0.2, 0.8];
%            robot.DataFormat = "row";
%
%            %Plan from start config to goal config
%            planner = manipulatorRRT(robot, env);
%            startConfig = [0.08, -0.65, 0.05, 0.02, 0.04, 0.49, 0.04];
%            goalConfig = [2.97, -1.05, 0.05, 0.02, 0.04, 0.49, 0.04];
%            path = plan(planner, startConfig, goalConfig);
%
%   References:
%       [1] J. Kuffner and S. LaValle, "RRT-connect: An efficient
%       approach to single-query path planning." In Proceedings of
%       IEEE International Conference on Robotics and Automation.
%       Vol.2, pp.995-1001, 2000
%
%   See also plannerRRT, plannerRRTStar

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

    properties
        %MaxConnectionDistance - Maximum length between planned configurations
        %   The length of the motion is computed as the Euclidean distance
        %   between two node configurations. Difference between two joint
        %   positions for a revolute joint with infinite joint limits is
        %   calculated using angDiff(). If the connect heuristic is enabled,
        %   then this property is ignored at the joining stage during planning.
        %
        %   Default: 0.1
        MaxConnectionDistance = 0.1;

        %MaxIterations- Maximum number of iterations
        %
        %   Default: 1e4
        MaxIterations = 1e4;

        %EnableConnectHeuristic - Directly join the start and goal trees
        %   When the connect heuristic is enabled, the extension process at the
        %   time of joining the two trees extends the tree directly to a
        %   configuration without accounting for MaxConnectionDistance.
        %
        %   Default: true
        EnableConnectHeuristic = true

        %ValidationDistance - Distance resolution for validating motion between configurations
        %   The validation distance determines the number of interpolated nodes
        %   between two adjacent nodes of the tree that should be checked for
        %   validity.
        %
        %   Default: 0.01
        ValidationDistance

        %WorkspaceGoalRegionBias - Probability to sample additional goal states within the workspace goal region
        %   The bias defines the probability to add additional goal states
        %   to the tree from the workspaceGoalRegion object specified in
        %   the range [0,1). When this value is set to zero, the
        %   workspaceGoalRegion object still samples a single goal for the
        %   planner to plan to.
        %   Increasing this value increases the likelihood of reaching a
        %   goal state in the goal region, but may lead to longer planning
        %   times because each new goal state adds additional complexity
        %   for planning.
        %
        %   Default: 0.50
        WorkspaceGoalRegionBias = 0.50;

        %IgnoreSelfCollision - Ignore self collisions during planning
        %   If this is property is set to true, self-colliding configurations will be considered
        %   valid in the output plan.
        %
        %   Default: false
        IgnoreSelfCollision = false;

        %SkippedSelfCollisions - Body pairs skipped for checking self-collisions
        %   Specify which pairs of bodies to ignore for self-collisions during
        %   planning. The property can take the following values:
        %
        %   - "parent" - Skip collision checking between child and parent
        %                bodies.
        %   - "adjacent" - Skip collision checking between bodies on adjacent
        %                  indices.
        %
        %   Default: 'parent'
        SkippedSelfCollisions = lower(char(robotics.manip.internal.SkippedSelfCollisionType.PARENT))
        
        ss
        sv
    end

    properties(Access=private)
        HasUserSetSkippedSelfCollisions = false
    end

    properties(Access=protected)
        %GoalRegionInternal - Concrete goal region of BiRRT
        GoalRegionInternal

    end

    methods
        function obj = manipulatorRRTSphere(robot_ec, robot_sc, environment, params)
        %MANIPULATORRRTSphere Constructor
%             ss = robotics.manip.internal.ManipulatorStateSpace(robot);
%             sv = ...
%                 robotics.manip.internal.ManipulatorStateValidator(...
%                 ss, environment, 0.01);
            ss = ManipulatorStateSpaceSphere(robot_ec, robot_sc);

            sv = ManipulatorStateValidatorSphere(...
                ss, environment, params.validationDistance, params.radius_offset, params);

            obj@matlabshared.planning.internal.BiRRT(sv);
            obj.ss = ss;
            obj.sv = sv;
            obj.GoalRegionInternal = workspaceGoalRegion(robot_sc.BodyNames{end});
            obj.GoalRegionBiasInternal = 0.5;
            obj.EnableConnectHeuristic = true;
            obj.IgnoreSelfCollision = false;
            obj.ValidationDistance = params.validationDistance;
        end

        function interpolatedPath = interpolate(obj, path, varargin)
        %interpolate Interpolate states along path
        %   INTERPOLATEDPATH = interpolate(PLANNER, PATH) interpolates
        %   between each adjacent configuration in the based on the
        %   ValidationDistance property.
        %
        %   INTERPOLATEDPATH = interpolate(PLANNER, PATH, NUMINTERPOLATIONS)
        %   NUMINTERPOLATIONS is the number of linear interpolations between
        %   each adjacent configuration in the path.
        %
        %   PATH is R-by-N matrix where R is the number of configurations,
        %   and N corresponds to the dimension of the configuration of the
        %   robot.

            narginchk(2, 3);
            if(nargin == 3)
                numInterpolations = varargin{1};
                interpolatedPath = ...
                    robotics.manip.internal.RRTUtils.interpolateByNumber(...
                    path, obj.StateValidatorInternal.StateSpace, numInterpolations);
            else
                interpolatedPath = ...
                    robotics.manip.internal.RRTUtils.interpolateByResolution(...
                    path, obj.StateValidatorInternal.StateSpace, obj.ValidationDistance);
            end
        end

        function shortenedPath = shorten(obj, path, numIterations)
        %shorten Trim edges to shorten the planned path
        %   SHORTENEDPATH = shorten(PLANNER, PATH, NUMITERATIONS) shortens
        %   the path by running a randomized shortening strategy for
        %   NUMITERATIONS number of iterations.
        %
        %   PATH is R-by-N matrix where R is the number of configurations,
        %   and N corresponds to the dimension of the configuration of the
        %   robot.
        %
        %   The shorten is a randomized shortening strategy repeated for the
        %   input number of iterations. With each iteration two random edges
        %   are selected and a point on each edge is connected. If the
        %   connection is valid, the intermediate edges are removed.

            obj.throwSkippedSelfCollisionsBehaviorChangeWarning();
            configSize = size(obj.StateValidatorInternal.StateSpace.JointBounds, 1);
            robotics.internal.validation.validateNumericMatrix(path, ...
                                                               'shorten', 'path', 'size',[NaN, configSize]);
            numIterations = ...
                robotics.internal.validation.validatePositiveIntegerScalar(numIterations,...
                                                                           'shorten',...
                                                                           'numIterations');
            shortenedPath = ...
                robotics.manip.internal.RRTUtils.shorten(...
                path, obj.StateValidatorInternal, numIterations);
        end

        function [path, solInfo] = plan(obj, startConfig, goal)
        %plan Plan motion from start to goal configuration
        %   [PATH, SOLINFO] = plan(PLANNER, STARTCONFIG, GOALCONFIG) plans
        %   a path between the start and goal configurations
        %
        %   [PATH, SOLINFO] = plan(PLANNER, STARTCONFIG, WORKSPACEGOALREGION) plans
        %   a path between the start configuration and a workspace goal
        %   region, specified as a workSpaceGoalRegion object.
        %
        %   The inputs STARTCONFIG and GOALCONFIG are 1-by-N vectors,
        %   where N is the dimension of the configuration of the
        %   robot.
        %
        %   The output PATH is a R-by-N matrix, where R is the number of
        %   planned configurations in the path.
        %
        %   The output SOLINFO is a struct with the following fields:
        %      IsPathFound  : Boolean indicating if a path was found
        %
        %      ExitFlag     : A number indicating why the planner terminated
        %                     1 - 'GoalReached'
        %                     2 - 'MaxIterationsReached'
        %
        %   See also workspaceGoalRegion

            obj.throwSkippedSelfCollisionsBehaviorChangeWarning();
            validateattributes(...
                goal, {'double', 'workspaceGoalRegion'}, {}, 'plan', 'goal');
%             obj.validateInputConfiguration(startConfig,
%             'StartConfiguration');  % Comment out by William since start
%             is already tested below
            if(isa(goal, 'workspaceGoalRegion'))
                obj.GoalRegionInternal = goal;
                obj.StateValidatorInternal.StateSpace.IKSolver.release();
                goalConfig = obj.sampleFromGoalRegionAndProject();
            else
                goalConfig = goal;

                %If planning to a single goal state, no need to sample from a
                %goal region, toggle to a "no goal region sampling" state.
                obj.GoalRegionBiasInternal = 0;
%                 obj.validateInputConfiguration(goalConfig,
%                 'GoalConfiguration');  % Comment out by William since
%                 goal is already tested below
            end

            % Ensure start/goal configs are valid
            invalidStartOrGoal = false;
            if ~obj.sv.isStateValid(startConfig) || ~obj.sv.isStateValid(goalConfig)
                path = [];
                disp('Start or Goal config is invalid (likely exceeds the sphere).')
            else
                [path, solInfo] = plan@matlabshared.planning.internal.BiRRT(obj, ...
                                                                        startConfig, goalConfig);
            end

            % Restore the state of the internal bias
            obj.GoalRegionBiasInternal = obj.WorkspaceGoalRegionBias;
        end

        function newObj = copy(obj)
        %copy Creates a copy of the planner
            robot = obj.StateValidatorInternal.StateSpace.RigidBodyTree;
            env = obj.StateValidatorInternal.Environment;
            newObj = manipulatorRRTSphere(robot_ec, robot_sc, env);
            newObj.ValidationDistance = obj.ValidationDistance;
            newObj.MaxIterations = obj.MaxIterations;
            newObj.EnableConnectHeuristic = obj.EnableConnectHeuristic;
            newObj.IgnoreSelfCollision = obj.IgnoreSelfCollision;
            newObj.MaxConnectionDistance = obj.MaxConnectionDistance;
            newObj.WorkspaceGoalRegionBias = obj.WorkspaceGoalRegionBias;
            newObj.SkippedSelfCollisions = obj.SkippedSelfCollisions;
        end

    end

    methods(Access=protected)
        function state = sampleFromGoalRegionAndProject(obj)
        %sampleFromGoalRegionAndProject Concrete template method of BiRRT
            state = ...
                obj.StateValidatorInternal.StateSpace.sampleGoalState(obj.GoalRegionInternal);
        end
    end

    methods
        function set.ValidationDistance(obj, val)
        %set.ValidationDistance Setter of ValidationDistance
            robotics.internal.validation.validatePositiveNumericScalar(val,...
                                                                       getClassName(obj),...
                                                                       'ValidationDistance');
            obj.ValidationDistance = val;
            obj.updateValidationDistance(val);
        end

        function set.MaxIterations(obj, maxIter)
        %set.MaxIterations Setter of MaxIterations
            robotics.internal.validation.validatePositiveIntegerScalar(maxIter,...
                                                                       getClassName(obj),...
                                                                       'MaxIterations');
            obj.MaxIterations = maxIter;
        end

        function set.MaxConnectionDistance(obj, maxConnDist)
        %set.MaxConnectionDistance Setter of MaxConnectionDistance
            robotics.internal.validation.validatePositiveNumericScalar(maxConnDist,...
                                                                       getClassName(obj), ...
                                                                       'MaxConnectionDistance');
            obj.MaxConnectionDistance = maxConnDist;
        end

        function set.EnableConnectHeuristic(obj, enableConnectHeuristic)
        %set.EnableConnectHeuristic Setter of EnableConnectHeuristic
            obj.EnableConnectHeuristic = ...
                robotics.internal.validation.validateLogical(enableConnectHeuristic,...
                                                             getClassName(obj), ...
                                                             'EnableConnectHeuristic');
        end

        function set.IgnoreSelfCollision(obj, ignoreSelfCollision)
        %set.IgnoreSelfCollision Setter of IgnoreSelfCollision
            obj.IgnoreSelfCollision = ...
                robotics.internal.validation.validateLogical(ignoreSelfCollision,...
                                                             getClassName(obj), ...
                                                             'IgnoreSelfCollision');
            obj.updateIgnoreSelfCollisionFlag(obj.IgnoreSelfCollision);
        end

        function set.SkippedSelfCollisions(obj,skippedSelfCollisions)
        %set.SkippedSelfCollisions Setter of SkippedSelfCollisions
            obj.SkippedSelfCollisions=validatestring(...
                skippedSelfCollisions, {...
                    lower(char(robotics.manip.internal.SkippedSelfCollisionType.ADJACENT)), ...
                    lower(char(robotics.manip.internal.SkippedSelfCollisionType.PARENT)), ...
                }, ...
                getClassName(obj),'SkippedSelfCollisions');
            obj.updateSkippedSelfCollisions(obj.SkippedSelfCollisions);
        end

        function set.WorkspaceGoalRegionBias(obj, workspaceGoalRegionBias)
        %set.GoalBias
            validateattributes(workspaceGoalRegionBias, {'double'}, ...
                               {'nonempty', 'scalar', 'real', 'nonnan', 'finite', '>=', 0.0, '<', 1.0}, ...
                               getClassName(obj), 'WorkspaceGoalRegionBias');
            obj.WorkspaceGoalRegionBias = workspaceGoalRegionBias;
            obj.GoalRegionBiasInternal = obj.WorkspaceGoalRegionBias;
        end

    end

    methods(Access = protected)

        function cname = getClassName(~)
        %getClassName Returns the name of the class
            cname = "manipulatorRRTSphere";
        end

        function validateInputConfiguration(obj, config, configTagInErrorID)
        %validateInputConfiguration Validates the input configuration
        %   Validates the input configuration and throws an appropriate error
        %   message. A configuration is valid when the underlying rigid
        %   body tree is self/world collision free.
            isColliding = ...
                obj.StateValidatorInternal.StateSpace.RigidBodyTree.checkCollision(...
                config, obj.StateValidatorInternal.Environment, ...
                'IgnoreSelfCollision', obj.convertFlagToOnOffSwitch(obj.IgnoreSelfCollision),...
                'SkippedSelfCollisions',obj.SkippedSelfCollisions);
            if(obj.IgnoreSelfCollision)
                isSelfColliding = false;
            else
                isSelfColliding = isColliding(1);
            end
            if(any(isColliding))
                if(isSelfColliding)
                    robotics.manip.internal.error(...
                        ['manipulatorplanning:' configTagInErrorID 'SelfColliding']);
                else
                    robotics.manip.internal.error(...
                        ['manipulatorplanning:' configTagInErrorID 'WorldColliding']);
                end
            end
        end
    end

    methods(Access=private)

        function updateValidationDistance(obj, val)
        %updateValidationDistance Updates the validation distance of the underlying StateValidator
            obj.StateValidatorInternal.ValidationDistance = val;
        end

        function updateIgnoreSelfCollisionFlag(obj, val)
        %updateIgnoreSelfCollisionFlag Updates the flag that ignores self collisions
        %   This updates the internal validator's self-collision checking flag
            obj.StateValidatorInternal.IgnoreSelfCollision = val;
        end

        function updateSkippedSelfCollisions(obj,val)
        %updateSkippedSelfCollisions Update the internal validator's SkippedSelfCollisions
            switch(val)
              case lower(char(robotics.manip.internal.SkippedSelfCollisionType.PARENT))
                skippedSelfCollisionType=robotics.manip.internal.SkippedSelfCollisionType.PARENT;
              otherwise
                skippedSelfCollisionType=robotics.manip.internal.SkippedSelfCollisionType.ADJACENT;
            end
            obj.StateValidatorInternal.SkippedSelfCollisionType=skippedSelfCollisionType;
            obj.HasUserSetSkippedSelfCollisions=true;
        end

        function throwSkippedSelfCollisionsBehaviorChangeWarning(obj)
        %throwSkippedSelfCollisionsBehaviorChangeWarning
        %   Warns the user about the default behavior change pertaining to
        %   self-collision checks during manipulatorRRTSphere/plan()
            if ~obj.IgnoreSelfCollision && ~obj.HasUserSetSkippedSelfCollisions
                    robotics.manip.internal.warning(...
                        'manipulatorplanning:SkippedSelfCollisionsUnset');
            end
        end


    end

    methods(Access=private, Static)
        function res = convertFlagToOnOffSwitch(flag)
        %convertFlagToOnOffSwitch Convert a boolean flag to 'on' or 'off' string
            if(flag)
                res = 'on';
            else
                res = 'off';
            end
        end
    end
end
