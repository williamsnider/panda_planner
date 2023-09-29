
function [x,y,z,h,success] = case3(A, m, aMax, vMax)
% Case 3 - h=aMax, y=vMax/h-x (max acceleration, max velocity)

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
y=vMax/h-x;
if abs(y) < eps(1e2)
    y = 0;
end
% Find z given A
% A = h*x^2 + 3/2*h*y*x + 1/2*h*y^2+z*h*x+z*h*y;
% z = (A - h*x^2 - 3/2*h*y*x - 1/2*h*y^2)/(h*x+h*y)
z = (A - h*x^2 - 3/2*h*y*x - 1/2*h*y^2)/(h*x+h*y);

if z>=0
    success=true;
end

    valid = checkProfile(x,y,z,h,A, vMax, aMax);
    assert(valid)
end
