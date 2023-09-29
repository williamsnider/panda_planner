% Code to move the robot into the ballpark of these estimated positions
% (will need to be verified manually)

clear; close all;
addpath("..")
params = CustomParameters();

%% Inputs
SAVE_DIR = "ballpark/";

disp("Shifting down Z slightly so that shape does not collide.")
%% Load robot
[panda_ec, panda_sc] = loadPandaWithShape();
env = build_collision_environment();

%% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree',panda_sc);
ik.SolverParameters.MaxIterations = 1000;
weights = [1 1 1 1 1 1];
ss = manipulatorStateSpace(panda_ec); % for joint limits
stateBounds = manipulatorStateSpace(panda_ec).StateBounds'; % for joint limits

%% Plot missing slots
load("missing_poses.mat", "missing_poses_map");
keys_list = missing_poses_map.keys;

% Plot shelf 00
pts_00 = [];
for key_num = 1:numel(keys_list)
    key = keys_list(key_num);
    key = key{1};

    if key(1:2) == "00"
        slot_pose = missing_poses_map(key);
        slot_xyz = slot_pose(1:3,4)';
        pts_00 = [pts_00;slot_xyz];
    end
end

% Plot shelf 12
pts_12 = [];
for key_num = 1:numel(keys_list)
    key = keys_list(key_num);
    key = key{1};

    if key(1:2) == "12"
        slot_pose = missing_poses_map(key);
        slot_xyz = slot_pose(1:3,4)';
        pts_12 = [pts_12;slot_xyz];
    end
end

% Plot all points
pts_all = [];
for key_num = 1:numel(keys_list)
    key = keys_list(key_num);
    key = key{1};

    slot_pose = missing_poses_map(key);
    slot_xyz = slot_pose(1:3,4)';
    pts_all = [pts_all;slot_xyz];

end


% plot3(pts_00(:,1),pts_00(:,2), pts_00(:,3), "g*"); hold on
% plot3(pts_12(:,1),pts_12(:,2), pts_12(:,3), "g*"); hold on
plot3(pts_all(:,1),pts_all(:,2), pts_all(:,3), "r*"); hold on

% Plot all found points
manual_files = dir("manual_slots");
manual_pts = [];
for i = 1:numel(manual_files)

    fname = manual_files(i).name;

    if contains(fname, ".txt") == false
        continue
    end

    [~, val] = readSlot("manual_slots/"+fname);
    point = val.O_T_EE;
    point = point(13:15)';
    manual_pts = [manual_pts;point];
end
% plot3(manual_pts(:,1),manual_pts(:,2), manual_pts(:,3), "g*"); hold on


%% Testing state validator with cylinders
ss = ManipulatorStateSpaceSphere(panda_ec, panda_sc);
sv = ManipulatorStateValidatorSphere(ss, env, params.validationDistance, params);
[q, val] = readSlot("manual_slots/"+"00A14.txt");
sv.isStateValid(q)
savename = "test";



% %% Cylinder -- Shelf 00
% vc1=[0,0,-0.3]';
% vc2=[0,0,-0.1]';
% d=2*params.radius_list(1);
% vp=[0,0,0]';
% I=CheckPointCylinder(vc1,vc2,d,vp);
% % Plot_Cylinder(vc1,vc2,d)
% % plot3(vp(1,I==0),vp(2,I==0),vp(3,I==0),'b.',vp(1,I==1),vp(2,I==1),vp(3,I==1),'r*');
% 
% %% Cylinder -- Shelf 12
% vc1=[0,0,0.9]';
% vc2=[0,0,1.1]';
% d=2*params.radius_list(13);
% vp=[0,0,0]';
% I=CheckPointCylinder(vc1,vc2,d,vp);
% % Plot_Cylinder(vc1,vc2,d)
% % plot3(vp(1,I==0),vp(2,I==0),vp(3,I==0),'b.',vp(1,I==1),vp(2,I==1),vp(3,I==1),'r*');


%% Plot robot
plotJointMotion(panda_sc, params.q_home, env, params)




% function Plot_Cylinder(vc1,vc2,d)
% Direction1(1,1)=vc2(1)-vc1(1);
% Direction1(2,1)=vc2(2)-vc1(2);
% Direction1(3,1)=vc2(3)-vc1(3);
% Direction1=Direction1/norm(Direction1);
% l1=Direction1(1,1); m1=Direction1(2,1); n1=Direction1(3,1);
% if abs(n1)<1e-6, n1=1e-6; end
% l2=rand(1); m2=rand(1); n2=(-l1*l2-m1*m2)/n1;
% Direction2=[l2 m2 n2];
% Direction2=Direction2/norm(Direction2);
% l3=m1*n2-m2*n1;
% m3=-l1*n2+l2*n1;
% n3=l1*m2-l2*m1;
% Direction3=[l3 m3 n3];
% Direction3=Direction3/norm(Direction3);
% for jk=1:1:12
%     Direction=Direction2*cos(2*pi*jk/12)-Direction3*sin(2*pi*jk/12);
%     xc1(jk)=vc1(1)+d/2*Direction(1);
%     yc1(jk)=vc1(2)+d/2*Direction(2);
%     zc1(jk)=vc1(3)+d/2*Direction(3);
%     xc2(jk)=vc2(1)+d/2*Direction(1);
%     yc2(jk)=vc2(2)+d/2*Direction(2);
%     zc2(jk)=vc2(3)+d/2*Direction(3);
% end
% plot3([xc1 xc1(1)],[yc1 yc1(1)],[zc1 zc1(1)],'k');
% hold on
% plot3([xc2 xc2(1)],[yc2 yc2(1)],[zc2 zc2(1)],'k');
% for jk=1:1:12
%     plot3([xc1(jk) xc2(jk)],[yc1(jk) yc2(jk)],[zc1(jk) zc2(jk)],'k');
% end
% daspect([1 1 1]);
% end
% 
