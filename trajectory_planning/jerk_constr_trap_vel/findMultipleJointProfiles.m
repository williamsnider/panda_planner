function allJointTrajectories = findMultipleJointProfiles(s0All, s1All,params)
%FINDMULTIPLEJOINTPROFILES Calculates the trapezoidal velocity profile
%given velocity, acceleration, and jerk constraints for multiple joints.
format long
CONTROL_HZ = 1000;
numJoints = numel(s0All);

% Test inputs
assert(all(params.vMaxAll>=0), "vMax must be entered as non-negative number.")
assert(all(params.aMaxAll>=0), "aMax must be entered as non-negative number.")
assert(all(params.jMaxAll>=0), "jMax must be entered as non-negative number.")

% Calculate time needed for each joint given constraints
minTimes = zeros(numJoints,1);
for i = 1:numJoints
    
    s1 = s1All(i);
    s0 = s0All(i);
    vMax = params.vMaxAll(i);
    aMax = params.aMaxAll(i);
    jMax = params.jMaxAll(i);
    disp(jMax)
    [accelRiseTime,accelConstTime,accelZeroTime,~] = findOptimalProfile(s1,s0,jMax, aMax, vMax);
    totalTime = 2*accelRiseTime + accelConstTime + accelZeroTime;
    minTimes(i) = totalTime;
end

% Use longest time, since it is the rate-limiting joint
T = max(minTimes);
T = ceil(T*1000)/1000;  % Force T to be multiple of control_hz

% Raise warning if T too large (causes matlab to crash)
if T > 1000
    allJointTrajectories = [];
    disp('T too large; returning empty array for allJointTrajectories')
    return
end

% Calculate time constrained profile
timesteps = 0:1/CONTROL_HZ:2*T;  % Note: T is time to reach HALF of total distance
allJointTrajectories = zeros(numJoints, numel(timesteps));
for i = 1:numJoints
    
    s1 = s1All(i);
    s0 = s0All(i);
    vMax = params.vMaxAll(i);
    aMax = params.aMaxAll(i);
    jMax = params.jMaxAll(i);
    [accelRiseTime,accelConstTime,accelZeroTime,peakAccel] = findOptimalProfileWithTimeConstraint(s1,s0,T,jMax, aMax, vMax);
    
    % Form this profile as piecewise polynomial
    segTimes = [accelRiseTime accelConstTime accelRiseTime accelZeroTime accelZeroTime accelRiseTime accelConstTime accelRiseTime];
    breaks = [0 cumsum(segTimes)];
    s = sign(peakAccel);
    m=s*jMax;
    h=peakAccel;
    coefs = [m 0; 0 h; -m h; 0 0; 0 0; -m 0; 0 -h; m -h];
    ppA = mkpp(breaks, coefs);
    
    % Get other profiles
    ppJ = fnder(ppA);
    ppV = fnint(ppA);
    ppX = fnint(ppV, s0);  % Add in starting value for position
%     plotProfiles(ppX, ppV, ppA, ppJ, vMax, aMax, jMax)
%     waitforbuttonpress()

    allJointTrajectories(i,:) = ppval(ppX,timesteps); % Note T = time to cover HALF of distance (symmetric)
end

% Pad beginning/ending to ensure velocity, acceleration, jerk = 0
allJointTrajectories = [repmat(allJointTrajectories(:,1), 1, 3), allJointTrajectories, repmat(allJointTrajectories(:,end), 1, 3)];
% plot_derivatives(allJointTrajectories(1:7,:)', params)
% waitforbuttonpress()
end

