function [x,y,z,h,success] = case1(A, m, aMax, vMax)
% Case 1 - z=0 and y=0 (no constant velocity / constant acceleration phase)

y=0;
z=0;

% Find h given A
% A = h*x^2
% A = h*(h/m)^2
% A = h^3 / m^2

% Handle A being negative
s = sign(A);
h = s*(abs(A)*m^2)^(1/3);
x=h/m;

% Test if velocity and acceleration constraints met
aPeak = h;
vPeak = h*(x+y);
if abs(aPeak)>abs(aMax) || abs(vPeak)>abs(vMax)
    success=false;
else
    success=true;
    valid = checkProfile(x,y,z,h,A, vMax, aMax);
    assert(valid)
end

end