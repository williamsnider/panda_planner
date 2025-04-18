function sv = construct_state_validator(robot_ec, robot_sc, env, params)
%CONSTRUCT_STATE_VALIDATOR Summary of this function goes here
%   Detailed explanation goes here
ss = ManipulatorStateSpaceEllipsoid(robot_ec, robot_sc);
ellipsoid_radius_offset = params.ellipsoid_radius_offset;  % Makes the ellipsoid slightly more conservative to improve robustness
sv = ManipulatorStateValidatorEllipsoid(ss, env, params.validationDistance, ellipsoid_radius_offset, params);
sv.IgnoreSelfCollision = false;
sv.Environment = env;


end

