function [x,y,z,h,success] = caseA(A, T, m, aMax, vMax)
% Case 1 - y=0 (no constant constant acceleration phase)

% Handle A being negative
s = sign(A);
y=0;

% A = 3hyx/2 + hy^2/2 + hx^2 + zhy + zhx
% T = x+y+z+x
% x = h/m
h_roots = roots([-1/m^2, T/m, 0, -A]);

if s==1
    h_roots = h_roots(h_roots>0);
    h = min(h_roots);
elseif s==-1
    h_roots = h_roots(h_roots<0);
    h = max(h_roots);
end

% Derived results
x = h/m;
z = T-2*x-y;

% Round z to 0 for very small negative Z's
if sign(z) == -1 && abs(z) < eps(1e3)
    z = 0;
end

% assert(x>=0);
% assert(y==0);
% assert(z>=0);

% Test if velocity and acceleration constraints met
valid = checkProfile(x,y,z,h,A, vMax, aMax);
if ~valid
    success=false;
else
    success=true;
    assert(valid);
end

end

