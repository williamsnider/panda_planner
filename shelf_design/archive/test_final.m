disp(num2str(i) + " of " + num2str(size(safety_combs,1)))
z = safety_combs(i,1);
radius = safety_combs(i,2);
theta = safety_combs(i,3);
[found_valid_cartesian_path, T, combined] = shelf_find_cartesian_path(z,radius,theta, SHAPE_ROTATION_ABOUT_Z, panda_ec, panda_sc, ik, weights, env, ss,ABOVE_HEIGHT, OUT_DIST, vMaxAll, aMaxAll, jMaxAll,vMaxAllAbsolute, aMaxAllAbsolute, jMaxAllAbsolute, jointMin, jointMax);
