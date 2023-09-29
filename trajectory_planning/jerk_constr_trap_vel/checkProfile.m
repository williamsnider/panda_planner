function valid = checkProfile(x,y,z,h,A, vMax, aMax)

nonnegativeX = x>=0;
nonnegativeY = y>=0;
nonnegativeZ = z>=0;
 
AccelBelowMax = abs(h)-abs(aMax)<=eps(1e7);
VelocityBelowMax = abs(h*(x+y))-abs(vMax)<= eps(1e7);
CorrectDistance = A-calcA(x,y,z,h)<eps(1e7);

if all([nonnegativeX, nonnegativeY, nonnegativeZ, AccelBelowMax, VelocityBelowMax, CorrectDistance])
    valid=true;
else
    valid=false;

% assert(x>=0)
% assert(y>=0)
% assert(z>=0)
% assert(abs(h)-abs(aMax)<=eps(1e5))
% assert(abs(h*(x+y))-abs(vMax)<= eps(1e5))
% assert(A-calcA(x,y,z,h)<eps(1e5))
end