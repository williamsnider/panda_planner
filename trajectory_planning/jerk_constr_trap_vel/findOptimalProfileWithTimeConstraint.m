function [accelRiseTime,accelConstTime,accelZeroTime,peakAccel] = findOptimalProfileWithTimeConstraint(s1, s0 ,T, jMax, aMax, vMax)
% FINDOPTIMALPROFILEWITHTIMECONSTRAINT This function is similar to
% findOptimalProfile, except that it also takes a time constraints input.
% It results in a profile that meets these contstraints: time (T), (vMax),
% accleration (aMax), and jerk (m) constraints. s0 is the starting
% position, and s1 is the goal position. It assumes that the time
% constraint (T) is longer than the minimum time needed to travel the
% distance.


A = (s1-s0)/2;

% Handle case where A = 0 (joint already in correct position)
if A == 0
    accelRiseTime = 0;
    accelConstTime = 0;
    accelZeroTime = T;  % Take full time
    peakAccel = 0;
    valid = checkProfile(accelRiseTime,accelConstTime,accelZeroTime,peakAccel,A, vMax, aMax);
    assert(valid)
    return
end


% Match sign
s = sign(A);
aMax = s*aMax;
vMax = s*vMax;
jMax = s*jMax;
m=jMax;  

assert(sign(A)==sign(aMax))
assert(sign(A)==sign(vMax))
% 
% Case 1 - no constant acceleration phase
[x,y,z,h,success] = caseA(A, T, m, aMax, vMax);

% Case 2 - Max acceleration, no constant velocity phase
if success==false
    [x,y,z,h,success] = caseB(A, T, m, aMax, vMax);
    checkProfile(x,y,z,h,A,vMax,aMax);
end

assert(success)

% Check results

accelRiseTime = x;
accelConstTime = y;
accelZeroTime = z;
peakAccel = h;
end