close all;
clear;

addpath([fileparts(mfilename('fullpath')), '/../..'])
run parameters.m

%% Inputs


start = [-0.0121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.01, 0.01];

startPose = [   0.999745294043658  -0.018297922043614  -0.013211097094657   0.366108911834217;
-0.018553650997199  -0.999637665573251  -0.019501271802996  -0.007218468784879;
-0.012849477508162   0.019741418797696  -0.999722545165218   0.330743061522072;
0                   0                   0   1.000000000000000];


goalPose = startPose;
goalPose(3,4) = goalPose(3,4) +0.15;

addpath([fileparts(mfilename('fullpath')), '/../robot'])
panda = loadPandaWithShape();

num_waypoints = 10;
window_size = 25;
[smoothed_q, con_q, con_t, T] = calc_cartesian_path(panda, startPose, goalPose, window_size, start, num_waypoints, vMaxAll, aMaxAll, jMaxAll);


[~, smoothed_qd, smoothed_qdd, smoothed_qddd] = calc_trajectory_derivatives(smoothed_q);


close all;
ax = subplot(2,4,1);
plot(smoothed_q')
title('x')

ax = subplot(2,4,2);
plot(smoothed_qd')
title('dx')

ax = subplot(2,4,3);
plot(smoothed_qdd')
title('ddx')

ax = subplot(2,4,4);
plot(smoothed_qddd')
title('dddx')

% Plot original trajectory vs smoothed
orig_xyz = [];
for i = 1:size(con_q,2)
    pose = getTransform(panda, con_q(:, i)', 'ee');
    point = pose(1:3,4)';
    orig_xyz = [orig_xyz; point];
end
smoothed_xyz = [];
for i = 1:size(smoothed_q,2)
    pose = getTransform(panda, smoothed_q(:, i)', 'ee');
    point = pose(1:3,4)';
    smoothed_xyz = [smoothed_xyz; point];
end

ax = subplot(2,4,5);
hold on;
plot(orig_xyz(:,1), orig_xyz(:,2),'b', 'DisplayName','orig')
plot(smoothed_xyz(:,1), smoothed_xyz(:,2),'g', 'DisplayName','smoothed')
for i = 1:num_waypoints
    plot(T(1, 4, i), T(2, 4, i), 'x', 'MarkerSize', 15)
end
xlabel('x')
ylabel('y')
title('xy')
daspect([1 1 1])
axis equal

ax = subplot(2,4,6);
hold on;
plot(orig_xyz(:,1), orig_xyz(:,3),'b', 'DisplayName','orig')
plot(smoothed_xyz(:,1), smoothed_xyz(:,3),'g', 'DisplayName','smoothed')
for i = 1:num_waypoints
    plot(T(1, 4, i), T(3, 4, i), 'x', 'MarkerSize', 15)
end
xlabel('x')
ylabel('z')
title('xz')
axis equal
daspect([1 1 1])

ax = subplot(2,4,7);
hold on;
plot(orig_xyz(:,2), orig_xyz(:,3),'b', 'DisplayName','orig')
plot(smoothed_xyz(:,2), smoothed_xyz(:,3),'g', 'DisplayName','smoothed')
for i = 1:num_waypoints
    plot(T(2, 4, i), T(3, 4, i), 'x', 'MarkerSize', 15)
end
xlabel('y')
ylabel('z')
title('yz')
axis equal
daspect([1 1 1])

ax = subplot(2,4,8);
hold on;
plot(con_t, orig_xyz(:,3),'b', 'DisplayName','orig')
smoothed_t = 0:0.001:(size(smoothed_xyz,1)-1)/1000;
plot(smoothed_t, smoothed_xyz(:,3),'g', 'DisplayName','smoothed')
xlabel('t')
ylabel('z')
legend

hold off;
figure
gca
addpath([fileparts(mfilename('fullpath')), '/../../visualization'])
plotJointMotion(panda,smoothed_q',{})


