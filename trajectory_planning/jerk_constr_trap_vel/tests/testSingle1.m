clear; close all;
addpath("..")

% Constraints
aMax = 1;
vMax = 1;
jMax = 10;

% Inputs
s1 = 10;
s0 = 20;

% Find optimal profile
[accelRiseTime,accelConstTime,accelZeroTime,peakAccel] = findOptimalProfile(s1, s0, jMax, aMax, vMax);


% Form this profile as piecewise polynomial
segTimes = [accelRiseTime accelConstTime accelRiseTime accelZeroTime accelZeroTime accelRiseTime accelConstTime accelRiseTime];
breaks = [0 cumsum(segTimes)];
coefs = [jMax 0; 0 peakAccel; -jMax peakAccel; 0 0; 0 0; -jMax 0; 0 -peakAccel; jMax -peakAccel];
ppA = mkpp(breaks, coefs);

% Get other profiles
ppJ = fnder(ppA);
ppV = fnint(ppA);
ppX = fnint(ppV);

plotProfiles(ppX, ppV, ppA, ppJ, vMax, aMax, jMax)
