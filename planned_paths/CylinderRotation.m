function R=CylinderRotation(a,b)
a=a/norm(a);
b=b/norm(b);
v=cross(a,b);
s=norm(v);
c=dot(a,b);
vx=[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
R=eye(3)+vx+vx*vx*(1-c)/s^2;
end