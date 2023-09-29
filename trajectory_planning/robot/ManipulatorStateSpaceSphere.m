classdef ManipulatorStateSpaceSphere < robotics.manip.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%ManipulatorStateSpaceSphere State space that captures the planning space of a rigid body tree

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

        
    properties(Access= public) % William - changed to public
        %RigidBodyTree The rigid body tree
        RigidBodyTree

        Robot_sc
        Robot_ec

        %WorkSpaceGoalRegion Goal region to sample a goal state from
        WorkSpaceGoalRegion;

        %IKSolver
        IKSolver

    end

    properties(SetAccess=private)
        %IsFreeRevolute Logical vector which encodes if a joint is a continuous or free revolute 
        %   The property encodes for a given configuration whether the joint
        %   value is of a revolute joint. This is used to ensure that operations
        %   related to angles are wrapped, accordingly. It is a row-vector of
        %   the size of robot's joint configuration.
        IsFreeRevolute

        %JointBounds The joint limits for a configuration corresponding to the RigidBodyTree
        JointBounds
    end

    methods
        function adjustJointBounds(obj, bounds)
            %adjustJointBounds Adjust the internal joint bounds based on the input bounds.
            %   Special handling is required for revolute and prismatic joints
            %   with inifinite limits.
            obj.JointBounds = bounds;
            obj.IsFreeRevolute = ...
                robotics.manip.internal.RigidBodyTreeUtils.identifyRevoluteJoint(obj.RigidBodyTree) & ...
                any(isinf(bounds), 2)';
            obj.clipToInfiniteLimit();
        end
    end


    properties(Constant)
        %InfiniteLimit The infinite limit for a specified joint bound.
        %   Joint bounds for non-wrapping joints (such as free revolute joints)
        %   that are set as inf will be set to InfiniteLimit to enable uniform
        %   sampling.
        InfiniteLimit = 1e10;
    end

    methods
        function obj = ManipulatorStateSpaceSphere(robot_ec, robot_sc)
        %ManipulatorStateSpaceSphere Constructor
        %   Constructs a ManipulatorStateSpaceSphere from a rigidBodyTree denoted
        %   by robot

        %Get configuration info from the robot
            [numNonFixedJoints, jointBounds] = ...
                robotics.manip.internal.RigidBodyTreeUtils.getConfigurationInfo(robot_sc);

            if(numNonFixedJoints == 0)
                robotics.manip.internal.error(...
                    'rigidbodytree:PlanningAtleastOneNonFixedJoint');
            end


            %Create the copy of robot as it is a handle object
            obj.RigidBodyTree = copy(robot_sc);
            obj.Robot_sc = copy(robot_sc);
            obj.Robot_ec = copy(robot_ec);

            %All the configurations are assumed to be in the 'row' data format.
            obj.RigidBodyTree.DataFormat = 'row';

            obj.adjustJointBounds(jointBounds);
            solverParam.AllowRandomRestart = false;

            % inverseKinematics is affecting the global random stream during
            % construction, hence, we will store the random seed's state and
            % restore the generator settings post construction of the
            % inverseKinematics object
            prevSeed = rng;
            obj.IKSolver = inverseKinematics("RigidBodyTree", obj.RigidBodyTree, ...
                                             "SolverParameters", solverParam);
            rng(prevSeed);
        end

        function state = sampleGoalState(obj, goalRegion)
        %sampleGoalState Samples a configuration from the underlying workspace goal region
            [state, info] = obj.IKSolver(goalRegion.EndEffectorName, ...
                                         goalRegion.sample(), ...
                                         ones(1, 6), ...
                                         obj.RigidBodyTree.homeConfiguration());

            %If IK solver fails to find a solution
            if(~strcmp(info.Status, 'success'))
                state = nan(size(state));
            end
        end

        function state = sampleUniform(obj, varargin)
        %sampleUniform Sample a configuration within the joint limits
        %   STATE = sampleUniform(obj) outputs a single random state.
        %
        %   STATE = sampleUniform(obj, NUMSAMPLES) outputs NUMSAMPLES number
        %   of random states. The output STATE is a NUMSAMPLES-by-N matrix
        %   where N denotes the dimension of the state (number of non-fixed
        %   joints).
        %
        %   STATE = sampleUniform(obj, NEARSTATE, DISTVEC, NUMSAMPLES) is
        %   not needed for this state space.

            numSamples = 1;
            lowerBound = obj.JointBounds(:, 1)';
            upperBound = obj.JointBounds(:, 2)';
            if(nargin == 2)
                numSamples = varargin{1};
            elseif(nargin > 2)
                nearState = varargin{1};
                distVec = varargin{2};
                numSamples = varargin{3};
                lowerBound = nearState - distVec;
                upperBound = nearState + distVec;
            end

            %Uniformly select numSamples value between 0 and 1 for each joint.
            randomRatio = rand(size(obj.JointBounds, 1), numSamples);

            %Return a random bounded configuration based on the following
            %pseudocode:
            %
            % state = lowerBound + randomRatio' .* (upperBound - lowerBound);
            %

            state = repmat(lowerBound, numSamples, 1) + ...
                    randomRatio' .* repmat(upperBound - lowerBound, numSamples, 1);
        end

        function interpState = interpolate(obj, state1, state2, ratio)
        %interpolate Interpolate between two states
        %   The interpolated states are obtained at the input RATIO value.
        %   RATIO can be a row vector in which case the output INTERPSTATE
        %   is matrix of size R-by-N where R is the number of elements in
        %   RATIO, and N is the number of non-fixed joints corresponding to
        %   the dimension of the state.

            stateDiff = state2 - state1;

            %Wrap the stateDiff for revolute joint values
            stateDiff(:, obj.IsFreeRevolute) = ...
                robotics.internal.wrapToPi(stateDiff(:, obj.IsFreeRevolute));

            %The interpolated state is a linear interpolation between state1 and state2
            interpState = repmat(state1, size(ratio, 2), 1) + ratio' * stateDiff;

            %Wrap the interpolated states for free revolute joint values
            interpState(:, obj.IsFreeRevolute) = ...
                robotics.internal.wrapToPi(interpState(:, obj.IsFreeRevolute));
        end

        function dist = distance(obj, state1, state2)
        %distance Distance between states
        %   DIST = distance(obj, STATE1, STATE2) computes the distance between
        %   STATE1 and STATE2 computed as a Euclidean norm of displacement
        %   between two joint positions. In case of a revolute joint with
        %   infinite limits, the difference in joint positions is wrapped to
        %   [-pi, pi] and then the norm is computed.  STATE1 and STATE2 should
        %   be matrices of compatible sizes.  When STATE1 and STATE2 are single
        %   states, the output DIST is a scalar.  When either of them is a
        %   R-by-N matrix (where R is the number of states, and N is the number
        %   of non-fixed joints),  the other state should be a 1-by-N or a
        %   R-by-N matrix, and in that case, DIST is a column vector of size R.

        %If state1 and state2 are compatible the operations below will cast
        %them into a common size.
            dist = robotics.manip.internal.RigidBodyTreeUtils.distance(...,
                state1, state2, obj.IsFreeRevolute);
        end
    end

    methods(Access=private)
        function clipToInfiniteLimit(obj)
        %clipToInfiniteLimit Clips the joint bound to +/-pi or +/-InfiniteLimit
        %   For a free revolute joint(a revolute joint with infinite joint
        %   limits) the bounds are clipped to +/-pi as 2pi or 0 are the same
        %   angles. Otherwise, for a non-wrapping joint or a prismatic joint, it
        %   is clipped to +/-InfiniteLimit
            obj.JointBounds(obj.IsFreeRevolute, 2) = pi;
            obj.JointBounds(obj.IsFreeRevolute, 1) = -pi;
            obj.JointBounds(find(obj.JointBounds > obj.InfiniteLimit)) = obj.InfiniteLimit;
            obj.JointBounds(find(obj.JointBounds < -obj.InfiniteLimit)) = -obj.InfiniteLimit;
        end
    end
end
