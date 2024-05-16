function has_collision = checkTrajForCollisions(sv, panda_sc, traj, params)

if checkTrajForEnvironmentCollisions(sv, traj, params) || checkTrajForSelfCollisions(panda_sc, traj, params)
    has_collision = true;
else
    has_collision = false;
end
end