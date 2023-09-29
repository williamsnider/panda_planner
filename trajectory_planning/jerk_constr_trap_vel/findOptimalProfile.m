function [accelRiseTime,accelConstTime,accelZeroTime,peakAccel] = findOptimalProfile(s1, s0 ,jMax, aMax, vMax)
% FINDOPTIMALPROFILE This function calculates the optimal profile given
% velocity (vMax), accleration (aMax), and jerk (m) constraints. s0 is the
% starting position, and s1 is the goal position.


A = (s1-s0)/2;

% Handle case where A = 0 (joint already in correct position)
if A == 0
    accelRiseTime = 0;
    accelConstTime = 0;
    accelZeroTime = 0;
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

% Case 1 - no constant acceleration, no constant velocity phase
[x,y,z,h,success] = case1(A, m, aMax, vMax);

% Case 2 - Max acceleration, no constant velocity phase
if success==false
    [x,y,z,h,success] = case2(A, m, aMax, vMax);
end

% Case 3 - Max acceleration, max velocity
if success==false
    [x,y,z,h,~] = case3(A, m, aMax, vMax);
end

% Check results
valid = checkProfile(x,y,z,h,A, vMax, aMax);
assert(valid)

accelRiseTime = x;
accelConstTime = y;
accelZeroTime = z;
peakAccel = h;
end