function sv = construct_state_validator(robot_ec, robot_sc, env, params)
%CONSTRUCT_STATE_VALIDATOR Summary of this function goes here
%   Detailed explanation goes here
ss = ManipulatorStateSpaceSphere(robot_ec, robot_sc);
radius_offset = params.radius_offset;  % Makes the sphere slightly more conservative to improve robustness
sv = ManipulatorStateValidatorSphere(ss, env, params.validationDistance, radius_offset, params);
sv.IgnoreSelfCollision = false;
sv.Environment = env;


end

