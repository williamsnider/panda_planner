function I=CheckPointCylinder(vc1,vc2,d,vp)
%move first coordinate of cylinder to origin
vc2=vc2-vc1;
vp=vp-vc1;
vc1=vc1-vc1;
%align the cylinder to x-axis or y-axis
x=[1 ; 0 ; 0];
if norm(cross(vc2,x))>1e-6
    R=CylinderRotation(vc2,x);
    vc2=R*vc2;
    vp=R*vp;
    I=vp(1,:)>=vc1(1,:) & vp(1,:)<=vc2(1,:) & (vp(2,:).^2+vp(3,:).^2)<=d^2/4;
else
    y=[0 ; 1 ; 0];
    R=CylinderRotation(vc2,y);
    vc2=R*vc2;
    vp=R*vp;
    I=vp(2,:)>=vc1(2,:) & vp(2,:)<=vc2(2,:) & (vp(1,:).^2+vp(3,:).^2)<=d^2/4;
end
end