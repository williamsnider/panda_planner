clear; close all;
addpath("..")

i=0;
while true
%     if mod(i,10000) == 0
%         disp(i)
%     end
    disp(i)
%     rng(i)
    i=i+1;
    % Constraints
    aMax = rand(1)*1e3+eps;
    vMax = rand(1)*1e3+eps;
    jMax = rand(1)*1e3+eps;
    
    % Inputs
    s1 = rand(1)*1e3;
    s0 = rand(1)*1e3;
    if (s1-s0==0)
        continue
    end
    
    % Find optimal profile
    [x,y,z,h] = findOptimalProfile(s1,s0, jMax, aMax, vMax);
    
    
    % Add some time to this
    T = 2*x+y+z;
    T = T+rand(1)*1e2+eps;
    [accelRiseTime,accelConstTime,accelZeroTime,peakAccel] = findOptimalProfileWithTimeConstraint(s1, s0 ,T, jMax, aMax, vMax);
%     % Form this profile as piecewise polynomial
%     segTimes = [x y x z z x y x];
%     breaks = [0 cumsum(segTimes)];
%     coefs = [m 0; 0 h; -m h; 0 0; 0 0; -m 0; 0 -h; m -h];
%     ppA = mkpp(breaks, coefs);
%     
%     % Get other profiles
%     ppJ = fnder(ppA);
%     ppV = fnint(ppA);
%     ppX = fnint(ppV);
% 
%     plotProfiles(ppX, ppV, ppA, ppJ, vMax, aMax, jMax)

end