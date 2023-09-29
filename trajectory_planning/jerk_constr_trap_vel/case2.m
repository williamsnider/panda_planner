
function [x,y,z,h,success] = case2(A, m, aMax, vMax)
% Case 2 - z=0, h=aMax (max acceleration, no constant velocity phase)
z=0;

% Calculate hMaxGivenVMax
s = sign(A);
hMaxGivenVMax = s*sqrt(m*vMax);

% Calculate hMaxGivenAMax
hMaxGivenAMax = aMax;

if s == 1
    h = min(hMaxGivenVMax, hMaxGivenAMax);
elseif s==-1
    h = max(hMaxGivenVMax, hMaxGivenAMax);
else
    error('A==0')
end
x=h/m;

% Find y given A
% A = h*x^2 + 3/2*h*y*x + 1/2*h*y^2
% 0 = 1/2*h*y^2 + 3/2*h*x*y + (h*x^2-A)
y = roots([1/2*h, 3/2*h*x, h*x^2-A]);
y = min(y(y>0)); % Get smallest positive root
assert(numel(y)==1)
% Test if max velocity constraint met
vPeak = h*(x+y);
if abs(vPeak) > abs(vMax)
    success=false;
else
    success=true;
valid = checkProfile(x,y,z,h,A, vMax, aMax);
assert(valid)
end
end