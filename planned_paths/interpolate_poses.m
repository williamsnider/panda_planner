



function [pose_map,center] = interpolate_poses(T_cell, idx_list, largest_slot)

% Check inputs
assert(strcmp(class(T_cell),class({})), "T_cell must be given as a cell array.")

% Check ascending idx_list
prev = -1;
for i = 1:numel(idx_list)
    idx = idx_list(i);
    assert(idx>prev, "idx_list must be ascending")
    prev = idx;
end

% for i = 1:numel(T_cell)
%     T = T_cell{i};
%     plotTransforms(se3(T)); hold on;
% end

%% Fit leftmost, middle, rightmost positions to circle
p1 = T_cell{1}(1:3,4)';
p2 = T_cell{2}(1:3,4)';
p3 = T_cell{3}(1:3,4)';
[center,rad,v1n,v2nb] = circlefit3d(p1,p2,p3);

% % Plot circle
% th = linspace(0,2*pi)';
% pts = rad*cos(th)*v1n + rad*sin(th)*v2nb + center;
% plot3(pts(:,1), pts(:,2), pts(:,3))

% Calculate theta for leftmost, middle, rightmost positions
xFun = @(th)myFun(th, p1, rad, v1n, v2nb, center);
th1 = fminbnd(xFun,0, 2*pi);

xFun = @(th)myFun(th, p2, rad, v1n, v2nb, center);
th2 = fminbnd(xFun, 0, 2*pi);

xFun = @(th)myFun(th, p3, rad, v1n, v2nb, center);
th3 = fminbnd(xFun, 0, 2*pi);


%% Points on circle
th_between_slots = (th3-th1)/(idx_list(3)-idx_list(1));
th_points = th1+ th_between_slots * (0:largest_slot)-idx_list(1)*th_between_slots;
points = calcPts(th_points', rad, v1n, v2nb, center);

points_map = containers.Map('KeyType','char','ValueType','any');
for idx = 0:largest_slot
    key = sprintf('%02d',idx);
    points_map(key) = points(idx+1,:);  % Correct for 0 shift
end

% Assert close to known positions
assert(norm(points_map(sprintf('%02d',idx_list(1)))-p1)<0.011);
assert(norm(points_map(sprintf('%02d',idx_list(2)))-p2)<0.011);
assert(norm(points_map(sprintf('%02d',idx_list(3)))-p3)<0.011);

%% Interpolate poses

% Extract original rotation matrix from pose
orig1 = T_cell{1}(1:3,1:3);
orig2 = T_cell{2}(1:3,1:3);
orig3 = T_cell{3}(1:3,1:3);

% Rotate poses by 10 degrees (corrects for shift on shelves)
rot1 = eul2rotm([pi/18, 0, 0], "ZYX") * orig1;
rot2 = eul2rotm([pi/18, 0, 0], "ZYX") * orig2;
rot3 = eul2rotm([pi/18, 0, 0], "ZYX") * orig3;


f = @(th_z)z_rot_func(th_z, rot1);
[z1,fval] = fminsearch(f, 0);

f = @(th_z)z_rot_func(th_z, rot2);
[z2,fval] = fminsearch(f, 0);

f = @(th_z)z_rot_func(th_z, rot3);
[z3,fval] = fminsearch(f, 0);

ang_between_slots = (z3-z1)/(idx_list(3) - idx_list(1));
ang_values = z1 + ang_between_slots * (0:largest_slot)-idx_list(1)*ang_between_slots;

rotations_map = containers.Map('KeyType','char','ValueType','any');
pose_map = containers.Map('KeyType','char','ValueType','any');
for idx = 0:largest_slot
    key = sprintf('%02d',idx);

    rot = eul2rotm([ang_values(idx+1), pi/2, 0]);
    rot = eul2rotm([-pi/18, 0, 0], "ZYX") * rot;  % Shift 10 degrees back

    rotations_map(key) = rot;

    pose = eye(4);
    pose(1:3,1:3) = rot;
    pose(1:3,4) = points_map(key);
    pose_map(key) =pose;

%     plotTransforms(se3(pose_map(key))); hold on;
end
% plotTransforms(se3(T_cell{1}), "FrameColor", "green"); hold on;
% plotTransforms(se3(T_cell{2}), "FrameColor", "green"); hold on;
% plotTransforms(se3(T_cell{3}), "FrameColor", "green"); hold on;
% 


% Check rotations
assert(dist(se3(rotations_map(sprintf('%02d',idx_list(1)))), se3(orig1))<0.03);
assert(dist(se3(rotations_map(sprintf('%02d',idx_list(2)))), se3(orig2))<0.03);
assert(dist(se3(rotations_map(sprintf('%02d',idx_list(3)))), se3(orig3))<0.03);

% Check poses
assert(dist(se3(pose_map(sprintf('%02d',idx_list(1)))), se3(T_cell{1}))<0.03);
assert(dist(se3(pose_map(sprintf('%02d',idx_list(2)))), se3(T_cell{2}))<0.03);
assert(dist(se3(pose_map(sprintf('%02d',idx_list(3)))), se3(T_cell{3}))<0.03);

% 
% % Plot points
% plot3(points(:,1), points(:,2), points(:,3), "k*"); hold on;
% plot3(p1(1), p1(2), p1(3), "go")
% plot3(p2(1), p2(2), p2(3), "go")
% plot3(p3(1), p3(2), p3(3), "go")
% plot3(points(idx_list(1)+1,1),points(idx_list(1)+1,2),points(idx_list(1)+1,3),"rx")
% plot3(points(idx_list(2)+1,1),points(idx_list(2)+1,2),points(idx_list(2)+1,3),"rx")
% plot3(points(idx_list(3)+1,1),points(idx_list(3)+1,2),points(idx_list(3)+1,3),"rx")
% axis("equal")

% %% Interpolate poses
% 
% % Left side
% num_left_slots = idx_list(2)+1;
% poses_left = interpolatePoses(T_cell{1}, T_cell{2}, idx_list(1), idx_list(2), th1, th2, num_left_slots,rad, v1n, v2nb, center);
% 
% % Right side
% num_right_slots = (largest_slot - idx_list(2)) + 1;
% poses_right = interpolatePoses(T_cell{2}, T_cell{3}, th2, th3, num_right_slots,rad, v1n, v2nb, center);
% 
% % plotTransforms(se3(poses_left));
% % hold on;
% % plotTransforms(se3(poses_right));
% 
% poses = cat(3, poses_left, poses_right(:,:,2:end));
% 
% % Test that middle poses calculated the same for both
% assert(all(max(abs(poses_left(:,:,end)-poses_right(:,:,1))<1e14)))
% 
% % Compare original vs calculated
% for i = 1:numel(idx_list)
%     T_orig = T_cell{i};
%     idx = idx_list(i)+1;
%     T_calc = poses(:,:,idx);
%     disp(i)
%     T_orig - T_calc
% 
%     plotTransforms(se3(T_orig), "FrameColor", "green"); hold on;
% %     plotTransforms(se3(T_calc))
% end
% 
% % Plot circle
% th = linspace(0,2*pi)';
% pts = rad*cos(th)*v1n + rad*sin(th)*v2nb + center;
% plot3(pts(:,1), pts(:,2), pts(:,3))

end


% function poses = interpolatePoses(pA, pB,idxA, idxB, thA, thB, num_slots, rad, v1n, v2nb, center, shelf_length)
% 
% % Divide this into two parts - calculating the point on the circle for each
% % pose and the rotation of each pose.
% 
% %% Points on circle
% th_between_slots = (thB-thA)/(idxB-idxA);
% th_points = thB - th_between_slots * (0:num_slots);
% points = calcPts(th_points', rad, v1n, v2nb, center);
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% % Sample points along circle at given th
% th_between_slots = (thB-thA)/(idxB-idxA);
% th_slots = thB - th_between_slots * (0:num_slots);
% % th_slots = linspace(thA, thB, num_slots)';
% pts = calcPts(th_slots, rad, v1n, v2nb, center);
% 
% 
% 
% 
% 
% % Rotation matrices
% rA = pA(1:3,1:3);
% rB = pB(1:3,1:3);
% 
% % Rotate by 10 degrees about Z-axis (adjusts for this shift in design)
% nA = eul2rotm([pi/18, 0, 0], "ZYX") * rA;
% nB = eul2rotm([pi/18, 0, 0], "ZYX") * rB;
% 
% % aA = pA;
% % aA(1:3,1:3) = eul2rotm([-thA, pi/2, 0]);
% % bA = pA;
% % bA(1:3,1:3) = nA;
% % plotTransforms(se3(aA)); hold on;
% % plotTransforms(se3(bA))
% % 
% % 
% % aB= pB;
% % aB(1:3,1:3) = eul2rotm([-thB, pi/2, 0]);
% % bB = pB;
% % bB(1:3,1:3) = nB;
% % plotTransforms(se3(aB)); hold on;
% % plotTransforms(se3(bB))
% 
% 
% rA = pA(1:3,1:3);
% pA_th = 1;
% plotTransforms(se3(pA)); hold on;
% plot3(0,0,0,"k*")
% 
% % Aligns to origin well
% rot = eul2rotm([pi/18, 0, 0], "ZYX") * rA;
% nA = pA;
% nA(1:3,1:3) = rot;
% [eul, eulalt] = tform2eul(nA)
% 
% xA = pA;
% xA(1:3,1:3) = eul2rotm([eul(1), pi/2, 0]);
% plotTransforms(se3(xA)); hold on;
% plotTransforms(se3(nA)); hold on;
% plot3(0,0,0,"k*")
% 
% % Interpolate
% 
% 
% % Given pose
% pA;
% 
% % Rotate by 10 degrees
% nA = pA;
% nA(1:3,1:3) = eul2rotm([pi/18, 0, 0], "ZYX") * pA(1:3,1:3);
% 
% % Extract theta
% [eul, eulalt] = rotm2eul(nA(1:3,1:3));
% pose_th = eul(1);
% 
% 
% 
% 
% 
% plot3(0,0,0,"k*")
% 
% 
% % Calculate theta from pose
% thC = rotm2eul(pA(1:3,1:3))
% % 
% % aA = pA;
% % aA(1:3,1:3) = nA;
% % bA = pA;
% % bA(1:3,1:3) = eul2rotm([-thA, pi/2, 0]);
% % plotTransforms(se3(aA)); hold on;
% % plotTransforms(se3(bA))
% % plot3(0,0,0, "k*")
% 
% 
% 
% % Quaternion
% qA = quaternion(rotm2quat(nA));
% qB = quaternion(rotm2quat(nB));
% 
% % Slerp
% th_scaled = (th_slots - th_slots(1))/(th_slots(end)-th_slots(1)); % scale to 0 and 1
% r_slerp = slerp(qA,qB, th_scaled);  % outputs quaternion
% T_slerp = quat2rotm(r_slerp);  % outputs rotation matrix
% 
% % Rotate by -10 degrees about Z-axis (undo above rotation)
% T_slots = repmat(eye(4), [1,1,num_slots]);
% for i = 1:size(T_slerp,3)
%     r = T_slerp(:,:,i);
%     n = eul2rotm([-pi/18, 0, 0], "ZYX") * r;
%     T_slots(1:3,1:3,i) = n;
% end
% T_slots(1:3,4,:) = pts';
% 
% poses = T_slots;
% end




function pts = calcPts(th, rad, v1n, v2nb, center)
pts = rad*cos(th)*v1n + rad*sin(th)*v2nb + center;
end

% Sample points along circle
function dist = myFun(th, p1, rad, v1n, v2nb, center)

point = rad*cos(th)*v1n + rad*sin(th)*v2nb + center;
dist = norm(p1-point,2);

end

function [center,rad,v1n,v2nb] = circlefit3d(p1,p2,p3)
% circlefit3d: Compute center and radii of circles in 3d which are defined by three points on the circumference
% usage: [center,rad,v1,v2] = circlefit3d(p1,p2,p3)
%
% arguments: (input)
%  p1, p2, p3 - vectors of points (rowwise, size(p1) = [n 3])
%               describing the three corresponding points on the same circle.
%               p1, p2 and p3 must have the same length n.
%
% arguments: (output)
%  center - (nx3) matrix of center points for each triple of points in p1,  p2, p3
%
%  rad    - (nx1) vector of circle radii.
%           if there have been errors, radii is a negative scalar ( = error code)
%
%  v1, v2 - (nx3) perpendicular vectors inside circle plane
%
% Example usage:
%
%  (1)
%      p1 = rand(10,3);
%      p2 = rand(10,3);
%      p3 = rand(10,3);
%      [center, rad] = circlefit3d(p1,p2,p3);
%      % verification, result should be all (nearly) zero
%      result(:,1)=sqrt(sum((p1-center).^2,2))-rad;
%      result(:,2)=sqrt(sum((p2-center).^2,2))-rad;
%      result(:,3)=sqrt(sum((p3-center).^2,2))-rad;
%      if sum(sum(abs(result))) < 1e-12,
%       disp('All circles have been found correctly.');
%      else,
%       disp('There had been errors.');
%      end
%
%
% (2)
%       p1=rand(4,3);p2=rand(4,3);p3=rand(4,3);
%       [center,rad,v1,v2] = circlefit3d(p1,p2,p3);
%       plot3(p1(:,1),p1(:,2),p1(:,3),'bo');hold on;plot3(p2(:,1),p2(:,2),p2(:,3),'bo');plot3(p3(:,1),p3(:,2),p3(:,3),'bo');
%       for i=1:361,
%           a = i/180*pi;
%           x = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1);
%           y = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2);
%           z = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3);
%           plot3(x,y,z,'r.');
%       end
%       axis equal;grid on;rotate3d on;
%
%
% Author: Johannes Korsawe
% E-mail: johannes.korsawe@volkswagen.de
% Release: 1.0
% Release date: 26/01/2012
% Default values
center = [];rad = 0;v1n=[];v2nb=[];
% check inputs
% check number of inputs
if nargin~=3,
    fprintf('??? Error using ==> cirlefit3d\nThree input matrices are needed.\n');rad = -1;return;
end
% check size of inputs
if size(p1,2)~=3 || size(p2,2)~=3 || size(p3,2)~=3,
    fprintf('??? Error using ==> cirlefit3d\nAll input matrices must have three columns.\n');rad = -2;return;
end
n = size(p1,1);
if size(p2,1)~=n || size(p3,1)~=n,
    fprintf('??? Error using ==> cirlefit3d\nAll input matrices must have the same number or rows.\n');rad = -3;return;
end
% more checks are to follow inside calculation
% Start calculation
% v1, v2 describe the vectors from p1 to p2 and p3, resp.
v1 = p2 - p1;v2 = p3 - p1;
% l1, l2 describe the lengths of those vectors
l1 = sqrt((v1(:,1).*v1(:,1)+v1(:,2).*v1(:,2)+v1(:,3).*v1(:,3)));
l2 = sqrt((v2(:,1).*v2(:,1)+v2(:,2).*v2(:,2)+v2(:,3).*v2(:,3)));
if find(l1==0) | find(l2==0), %#ok<OR2>
    fprintf('??? Error using ==> cirlefit3d\nCorresponding input points must not be identical.\n');rad = -4;return;
end
% v1n, v2n describe the normalized vectors v1 and v2
v1n = v1;for i=1:3, v1n(:,i) = v1n(:,i)./l1;end
v2n = v2;for i=1:3, v2n(:,i) = v2n(:,i)./l2;end
% nv describes the normal vector on the plane of the circle
nv = [v1n(:,2).*v2n(:,3) - v1n(:,3).*v2n(:,2) , v1n(:,3).*v2n(:,1) - v1n(:,1).*v2n(:,3) , v1n(:,1).*v2n(:,2) - v1n(:,2).*v2n(:,1)];
if find(sum(abs(nv),2)<1e-5),
    fprintf('??? Warning using ==> cirlefit3d\nSome corresponding input points are nearly collinear.\n');
end
% v2nb: orthogonalization of v2n against v1n
dotp = v2n(:,1).*v1n(:,1) + v2n(:,2).*v1n(:,2) + v2n(:,3).*v1n(:,3);
v2nb = v2n;for i=1:3,v2nb(:,i) = v2nb(:,i) - dotp.*v1n(:,i);end
% normalize v2nb
l2nb = sqrt((v2nb(:,1).*v2nb(:,1)+v2nb(:,2).*v2nb(:,2)+v2nb(:,3).*v2nb(:,3)));
for i=1:3, v2nb(:,i) = v2nb(:,i)./l2nb;end
% remark: the circle plane will now be discretized as follows
%
% origin: p1                    normal vector on plane: nv
% first coordinate vector: v1n  second coordinate vector: v2nb
% calculate 2d coordinates of points in each plane
% p1_2d = zeros(n,2); % set per construction
% p2_2d = zeros(n,2);p2_2d(:,1) = l1; % set per construction
p3_2d = zeros(n,2); % has to be calculated
for i = 1:3,
    p3_2d(:,1) = p3_2d(:,1) + v2(:,i).*v1n(:,i);
    p3_2d(:,2) = p3_2d(:,2) + v2(:,i).*v2nb(:,i);
end
% calculate the fitting circle
% due to the special construction of the 2d system this boils down to solving
% q1 = [0,0], q2 = [a,0], q3 = [b,c] (points on 2d circle)
% crossing perpendicular bisectors, s and t running indices:
% solve [a/2,s] = [b/2 + c*t, c/2 - b*t]
% solution t = (a-b)/(2*c)
a = l1;b = p3_2d(:,1);c = p3_2d(:,2);
t = 0.5*(a-b)./c;
scale1 = b/2 + c.*t;scale2 = c/2 - b.*t;
% centers
center = zeros(n,3);
for i=1:3,
    center(:,i) = p1(:,i) + scale1.*v1n(:,i) + scale2.*v2nb(:,i);
end
% radii
rad = sqrt((center(:,1)-p1(:,1)).^2+(center(:,2)-p1(:,2)).^2+(center(:,3)-p1(:,3)).^2);
end




function output_dist = z_rot_func(th_z, rot)
new_rot = eul2rotm([th_z, pi/2, 0]);
output_dist = dist(se3(rot), se3(new_rot));
end

