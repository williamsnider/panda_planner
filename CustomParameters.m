classdef CustomParameters


 properties
 %% Place all non-derived parameters here

 % Shelf dimensions
 DIST_FINGERTIP_ABOVE_SHELF = 0.0225
 DIST_FINGERTIP_FROM_INNER_SHELF = 0.015 + 0.0254/2; % inner thickness of shelf + half of slot dimension
 SHELF_THICKNESS = 0.0254/2 % 1/2 inch

 % Cartesian path
 ABOVE_HEIGHT = 0.0202;
 SHAPE_ROTATION_ABOUT_Z = pi/18;

 % Joint Space kinematic constraint scaling factors
 vScale = 0.7;
 aScale = 0.8;
 jScale = 0.1;

 % Cartesian space kinematic constraitn scaling factors
 vScaleCart = 0.5;
 aScaleCart = 0.2;
 jScaleCart = 0.1;

 % Cartesian paths
 num_waypoints = 20; % How densely sampled a cartesian path is
 window_size = 50; % Sliding window average for cartesian paths (reduces jerk)

 % Path planning
 validationDistance = 0.01 % Used in RRT planner
 checkSteps = 250  % ms; every 250ms is checked by motionCheck for environmental collisions
 checkProportion = 0.05 % Proportion of states that are double checked for kinematic constraints and self/environment collisiosn
 smoothing_window_size = 100 % Smooths out joint-to-joint motions
    multiMovementPadding = 50  % For a trajectory with multiple steps, pad the start/end of each step by this amount to smooth the transitions
 
    % Joint positions
 q_home = [-0.0121707, -0.561084, 0.00127942, -2.60702, -0.0211893, 2.03285, 0.802306, 0.01, 0.01];
 
 
 % Model held shapes as cylinders
 cylinderRadius = 0.025;  % Length of the cylinder
 cylinderLength = 0.05;  % Radius of the cylinder
 %
 %  monkeyXYZ = [-0.62 -0.0 0.40]; % from Neo's perspective +X: towards far wall, +Y: towards left, +Z: towards ceiling
 %  pullXYZ = [-0.62 -0.125 0.40];
 %  startToMonkeyTime = 1.0;
 %  monkeyHoldTime = 2.0;
 %  monkeyToPullTime = 3.0;
 %  pullToStartTime = 1.0;
 %  poseVecLength = 16;
 %  weights = [0.25 0.25 0.25 1 1 1];

 % Visualization
 dispHz = 15; % Much higher does not plot at correct speed
 rateCtrlObj
 %  eulHigherSlots = [0 pi/2 0]; % blue out, red down
 %  eulLowerSlots = [0 3*pi/2 pi]; % blue out red up


 % Staging box - this is the region near the staging area where there are
 % no shelves. Because the sphere assumes there would be a shelf, including
 % this separate area as part of manipulatorstatevalidatorsphere lets paths
 % be planned here. It is important to keep an additional collision object
 % representing the monkey's arm reaching port to avoid colliding with
 % that. That should be placed in build_collision_environment.m
staging_box_width = 0.2; 
staging_box_height = 0.7; 
staging_box_length = 1.0; 
staging_box_center = [-0.72, 0.0, 0.325]; % Replace with the desired center coordinates [x, y, z]
            


 Z_list
 radius_list
 sphere_radius
 radius_offset
 sphere_buffer
 sphere_origin
 sphere_cutoff_bottom
 sphere_cutoff_top
 cylinder_height
 cylinder_buffer
 bottom_cylinder_radius
 top_cylinder_radius
 shelf_pts

 % Directories
 CustomParametersDir
 slots_for_sphere_dir

 %% Do not edit below this line %%

 % Velocity, acceleration, jerk constraints for each joint
 % (fake values for last 2, which are the gripper, but the real robot is
 % controlled differently).

 % translate from panda_hand to panda_hand_tcp
 hand_to_tcp = 0.1034;

 % Joint kinematic constraints (fake values for last two, which are gripper,
 % but real robot is controlled differently).
 sMaxAllAbsolute = [2.8973 1.7628 2.8973 -0.0698 2.8973 3.7525 2.8973 10.0 10.0];
 sMinAllAbsolute = [-2.8973 -1.7628 -2.8973 -3.0718 -2.8973 -0.0175 -2.8973 0.0 0.0];
 vMaxAllAbsolute = [2.1750	2.1750	2.1750	2.1750	2.6100	2.6100	2.6100 100.0 100.0];
 aMaxAllAbsolute = [15	7.5	10	12.5	15	20	20 100.0 100.0];
 jMaxAllAbsolute = [7500,3750,5000,6250,7500,10000,10000,1000000,1000000];
 vMaxAll
 aMaxAll
 jMaxAll

 % Cartesian kinematic constraints
 vMaxCartTransAbsolute = 1.7;
 aMaxCartTransAbsolute = 13.0;
 jMaxCartTransAbsolute = 6500.0;
 vMaxCartRotAbsolute = 2.5;
 aMaxCartRotAbsolute = 25.0;
 jMaxCartRotAbsolute = 12500.0;
 vMaxCartTrans
 aMaxCartTrans
 jMaxCartTrans
 vMaxCartRot
 aMaxCartRot
 jMaxCartRot

 % Joint limits (only used for plotting; RBT joint limits set in .urdf)
 jointMax = [2.8973 1.7628 2.8973 -0.0698 2.8973 3.7525 2.8973];
 jointMin = [-2.8973 -1.7628 -2.8973 -3.0718 -2.8973 -0.0175 -2.8973];

 shelf_names = ["00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12"];
 shelf_lengths = [41,47,50,51,52,52,52,51,50,47,44,39,32]; % This is already 0-indexed. E.G. shelf_0 has slots 00 to 47 (48 total).
 panel_names = ["A", "B","C"];


 % Staging positions - calculated in find_staging.m
staging_A0 = [1.7389       -1.05      1.1504     -2.2258    -0.67158      2.1635    -0.75842        0.01        0.01];
extreme_A0 = [1.228     -1.0527      1.4563     -1.6884    -0.50592       2.072    -0.83151        0.01        0.01];
staging_A1 = [1.7389       -1.05      1.1504     -2.2258    -0.67158      2.1635     0.81237        0.01        0.01];
extreme_A1 = [1.228     -1.0527      1.4563     -1.6884    -0.50592       2.072     0.73929        0.01        0.01];
staging_A2 = [1.7389       -1.05      1.1504     -2.2258    -0.67158      2.1635      2.3832        0.01        0.01];
extreme_A2 = [1.228     -1.0527      1.4563     -1.6884    -0.50592       2.072      2.3101        0.01        0.01];
staging_A3 = [1.7389       -1.05      1.1504     -2.2258    -0.67158      2.1635     -2.3292        0.01        0.01];
extreme_A3 = [1.228     -1.0527      1.4563     -1.6884    -0.50592       2.072     -2.4023        0.01        0.01];
staging_B0 = [1.7146     0.70955     0.99884     -2.2867      2.1275     0.70133     -1.1127        0.01        0.01];
extreme_B0 = [2.0775     0.97271     0.91356     -1.7493      2.1881     0.93122     -1.0508        0.01        0.01];
staging_B1 = [1.7146     0.70955     0.99884     -2.2867      2.1275     0.70133     0.45807        0.01        0.01];
extreme_B1 = [2.0775     0.97271     0.91356     -1.7493      2.1881     0.93122     0.52001        0.01        0.01];
staging_B2 = [1.7146     0.70955     0.99884     -2.2867      2.1275     0.70133      2.0289        0.01        0.01];
extreme_B2 = [2.0775     0.97271     0.91356     -1.7493      2.1881     0.93122      2.0908        0.01        0.01];
staging_B3 = [1.7146     0.70955     0.99884     -2.2867      2.1275     0.70133     -2.6835        0.01        0.01];
extreme_B3 = [2.0775     0.97271     0.91356     -1.7493      2.1881     0.93122     -2.6216        0.01        0.01];
staging_C0 = [1.8813     -1.0161     0.51292     -2.6957      2.0971      2.0901      2.1758        0.01        0.01];
extreme_C0 = [1.6839    -0.67159     0.90602     -2.3099      2.3261       2.299      2.0236        0.01        0.01];
staging_C1 = [1.8813     -1.0161     0.51292     -2.6957      2.0971      2.0901     -2.5366        0.01        0.01];
extreme_C1 = [1.6839    -0.67159     0.90602     -2.3099      2.3261       2.299     -2.6888        0.01        0.01];
staging_C2 = [1.8813     -1.0161     0.51292     -2.6957      2.0971      2.0901     -0.9658        0.01        0.01];
extreme_C2 = [1.6839    -0.67159     0.90602     -2.3099      2.3261       2.299      -1.118        0.01        0.01];
staging_C3 = [1.8813     -1.0161     0.51292     -2.6957      2.0971      2.0901     0.60499        0.01        0.01];
extreme_C3 = [1.6839    -0.67159     0.90602     -2.3099      2.3261       2.299     0.45281        0.01        0.01];
staging_D0 = [1.9714     -1.0599      1.0209     -2.6602     -2.0224      2.3689      1.1169        0.01        0.01];
extreme_D0 = [1.4962    -0.91794      1.3852     -2.2112     -1.8942      2.3306      1.0512        0.01        0.01];
staging_D1 = [1.9714     -1.0599      1.0209     -2.6602     -2.0224      2.3689      2.6877        0.01        0.01];
extreme_D1 = [1.4962    -0.91794      1.3852     -2.2112     -1.8942      2.3306       2.622        0.01        0.01];
staging_D2 = [1.9714     -1.0599      1.0209     -2.6602     -2.0224      2.3689     -2.0247        0.01        0.01];
extreme_D2 = [1.4962    -0.91794      1.3852     -2.2112     -1.8942      2.3306     -2.0904        0.01        0.01];
staging_D3 = [1.9714     -1.0599      1.0209     -2.6602     -2.0224      2.3689    -0.45389        0.01        0.01];
extreme_D3 = [1.4962    -0.91794      1.3852     -2.2112     -1.8942      2.3306    -0.51964        0.01        0.01];
staging_X0 = [-1.2969    -0.48259      -1.662     -2.4539     -2.5873     0.58612    -0.46759        0.01        0.01];
extreme_X0 = [-0.58363    -0.57121     -2.3614     -1.8999     -2.5873     0.80958    -0.49249        0.01        0.01];
staging_X1 = [-1.2969    -0.48259      -1.662     -2.4539     -2.5873     0.58612      1.1032        0.01        0.01];
extreme_X1 = [-0.58363    -0.57121     -2.3614     -1.8999     -2.5873     0.80958      1.0783        0.01        0.01];
staging_X2 = [-1.2969    -0.48259      -1.662     -2.4539     -2.5873     0.58612       2.674        0.01        0.01];
extreme_X2 = [-0.58363    -0.57121     -2.3614     -1.8999     -2.5873     0.80958      2.6491        0.01        0.01];
staging_X3 = [-1.2969    -0.48259      -1.662     -2.4539     -2.5873     0.58612     -2.0384        0.01        0.01];
extreme_X3 = [-0.58363    -0.57121     -2.3614     -1.8999     -2.5873     0.80958     -2.0633        0.01        0.01];
staging_Y0 = [-2.2508     -1.1383    -0.64365     -2.7077     -2.1602      1.7207    -0.70947        0.01        0.01];
extreme_Y0 = [-1.896    -0.75342     -1.0068     -2.2965     -2.2558      1.8083    -0.56711        0.01        0.01];
staging_Y1 = [-2.2508     -1.1383    -0.64365     -2.7077     -2.1602      1.7207     0.86133        0.01        0.01];
extreme_Y1 = [-1.896    -0.75342     -1.0068     -2.2965     -2.2558      1.8083      1.0037        0.01        0.01];
staging_Y2 = [-2.2508     -1.1383    -0.64365     -2.7077     -2.1602      1.7207      2.4321        0.01        0.01];
extreme_Y2 = [-1.896    -0.75342     -1.0068     -2.2965     -2.2558      1.8083      2.5745        0.01        0.01];
staging_Y3 = [-2.2508     -1.1383    -0.64365     -2.7077     -2.1602      1.7207     -2.2803        0.01        0.01];
extreme_Y3 = [-1.896    -0.75342     -1.0068     -2.2965     -2.2558      1.8083     -2.1379        0.01        0.01];
staging_Z0 = [0.61919       1.028      1.9332     -2.7535      2.2828      1.9657      0.5536        0.01        0.01];
extreme_Z0 = [1.2316     0.91623      1.6334     -2.3408      2.3621      1.9173     0.45162        0.01        0.01];
staging_Z1 = [0.61919       1.028      1.9332     -2.7535      2.2828      1.9657      2.1244        0.01        0.01];
extreme_Z1 = [1.2316     0.91623      1.6334     -2.3408      2.3621      1.9173      2.0224        0.01        0.01];
staging_Z2 = [0.61919       1.028      1.9332     -2.7535      2.2828      1.9657      -2.588        0.01        0.01];
extreme_Z2 = [1.2316     0.91623      1.6334     -2.3408      2.3621      1.9173       -2.69        0.01        0.01];
staging_Z3 = [0.61919       1.028      1.9332     -2.7535      2.2828      1.9657     -1.0172        0.01        0.01];
extreme_Z3 = [1.2316     0.91623      1.6334     -2.3408      2.3621      1.9173     -1.1192        0.01        0.01];
%  stagingA0 = [1.8926 0.47805 0.61475 -1.55 1.0511 1.0302 1.9264 0.01 0.01]
%  extremeA0 = [2.2823 1.0276 0.62193 -0.70516 1.0185 1.1003 1.9264 0.01 0.01]
%  stagingA1 = [1.8926 0.47805 0.61475 -1.55 1.0511 1.0302 -2.786 0.01 0.01]
%  extremeA1 = [2.2823 1.0276 0.62193 -0.70516 1.0185 1.1003 -2.786 0.01 0.01]
%  stagingA2 = [1.8926 0.47805 0.61475 -1.55 1.0511 1.0302 -1.2152 0.01 0.01]
%  extremeA2 = [2.2823 1.0276 0.62193 -0.70516 1.0185 1.1003 -1.2152 0.01 0.01]
%  stagingA3 = [1.8926 0.47805 0.61475 -1.55 1.0511 1.0302 0.35562 0.01 0.01]
%  extremeA3 = [2.2823 1.0276 0.62193 -0.70516 1.0185 1.1003 0.35562 0.01 0.01]
%  stagingB0 = [-1.4749 -1.0338 -1.0289 -1.562 -0.96709 1.1066 -1.9696 0.01 0.01]
%  extremeB0 = [-0.74189 -0.94155 -1.471 -0.69496 -1.1976 1.043 -1.9696 0.01 0.01]
%  stagingB1 = [-1.4749 -1.0338 -1.0289 -1.562 -0.96709 1.1066 -0.39879 0.01 0.01]
%  extremeB1 = [-0.74189 -0.94155 -1.471 -0.69496 -1.1976 1.043 -0.39879 0.01 0.01]
%  stagingB2 = [-1.4749 -1.0338 -1.0289 -1.562 -0.96709 1.1066 1.172 0.01 0.01]
%  extremeB2 = [-0.74189 -0.94155 -1.471 -0.69496 -1.1976 1.043 1.172 0.01 0.01]
%  stagingB3 = [-1.4749 -1.0338 -1.0289 -1.562 -0.96709 1.1066 2.7428 0.01 0.01]
%  extremeB3 = [-0.74189 -0.94155 -1.471 -0.69496 -1.1976 1.043 2.7428 0.01 0.01]
%  stagingC0 = [1.7409 -0.78281 1.3487 -2.206 -0.90362 1.986 -0.45649 0.01 0.01]
%  extremeC0 = [1.1382 -0.89493 1.7443 -1.6486 -0.74744 1.9487 -0.45649 0.01 0.01]
%  stagingC1 = [1.7409 -0.78281 1.3487 -2.206 -0.90362 1.986 1.1143 0.01 0.01]
%  extremeC1 = [1.1382 -0.89493 1.7443 -1.6486 -0.74744 1.9487 1.1143 0.01 0.01]
%  stagingC2 = [1.7409 -0.78281 1.3487 -2.206 -0.90362 1.986 2.6851 0.01 0.01]
%  extremeC2 = [1.1382 -0.89493 1.7443 -1.6486 -0.74744 1.9487 2.6851 0.01 0.01]
%  stagingC3 = [1.7409 -0.78281 1.3487 -2.206 -0.90362 1.986 -2.0273 0.01 0.01]
%  extremeC3 = [1.1382 -0.89493 1.7443 -1.6486 -0.74744 1.9487 -2.0273 0.01 0.01]
%  stagingD0 = [0.90215 -1.1957 2.0076 -2.1514 -1.6776 1.0121 -1.2218 0.01 0.01]
%  extremeD0 = [0.64609 -1.3544 2.0138 -1.5975 -1.7974 1.1327 -1.2218 0.01 0.01]
%  stagingD1 = [0.90215 -1.1957 2.0076 -2.1514 -1.6776 1.0121 0.349 0.01 0.01]
%  extremeD1 = [0.64609 -1.3544 2.0138 -1.5975 -1.7974 1.1327 0.349 0.01 0.01]
%  stagingD2 = [0.90215 -1.1957 2.0076 -2.1514 -1.6776 1.0121 1.9198 0.01 0.01]
%  extremeD2 = [0.64609 -1.3544 2.0138 -1.5975 -1.7974 1.1327 1.9198 0.01 0.01]
%  stagingD3 = [0.90215 -1.1957 2.0076 -2.1514 -1.6776 1.0121 -2.7926 0.01 0.01]
%  extremeD3 = [0.64609 -1.3544 2.0138 -1.5975 -1.7974 1.1327 -2.7926 0.01 0.01]
%  stagingE0 = [2.4253 -0.70044 0.29238 -2.6872 2.3456 2.4375 1.658 0.01 0.01]
%  extremeE0 = [2.3466 -0.21794 0.49534 -2.2397 2.5636 2.5535 1.658 0.01 0.01]
%  stagingE1 = [2.4253 -0.70044 0.29238 -2.6872 2.3456 2.4375 -3.0544 0.01 0.01]
%  extremeE1 = [2.3466 -0.21794 0.49534 -2.2397 2.5636 2.5535 -3.0544 0.01 0.01]
%  stagingE2 = [2.4253 -0.70044 0.29238 -2.6872 2.3456 2.4375 -1.4836 0.01 0.01]
%  extremeE2 = [2.3466 -0.21794 0.49534 -2.2397 2.5636 2.5535 -1.4836 0.01 0.01]
%  stagingE3 = [2.4253 -0.70044 0.29238 -2.6872 2.3456 2.4375 0.087227 0.01 0.01]
%  extremeE3 = [2.3466 -0.21794 0.49534 -2.2397 2.5636 2.5535 0.087227 0.01 0.01]
 end

 methods

 % Place all derived parameters and function calls here
 function obj = CustomParameters()

  run add_all_paths.m

  % Check that velocity, acceleration, and jerk constraints not exceeded.
  assert(obj.vScale<0.99 && obj.vScale>0.0, "WARNING: vScale is not in interval (0, 0.99)")
  assert(obj.aScale<0.99 && obj.aScale>0.0, "WARNING: aScale is not in interval (0, 0.99)")
  assert(obj.jScale<0.99 && obj.jScale>0.0, "WARNING: vScale is not in interval (0, 0.99)")

  % For visualization
  obj.rateCtrlObj = rateControl(obj.dispHz);
  obj.vMaxAll = obj.vMaxAllAbsolute*obj.vScale;
  obj.aMaxAll = obj.aMaxAllAbsolute*obj.aScale;
  obj.jMaxAll = obj.jMaxAllAbsolute*obj.jScale;


  obj.vMaxCartTrans = obj.vMaxCartTransAbsolute*obj.vScaleCart;
  obj.aMaxCartTrans = obj.aMaxCartTransAbsolute*obj.aScaleCart;
  obj.jMaxCartTrans = obj.jMaxCartTransAbsolute*obj.jScaleCart;
  obj.vMaxCartRot = obj.vMaxCartRotAbsolute;
  obj.aMaxCartRot = obj.aMaxCartRotAbsolute;
  obj.jMaxCartRot = obj.jMaxCartRotAbsolute;

  [obj.CustomParametersDir,~,~] = fileparts(mfilename("fullpath")); % Located in trajectory_planning/collision_environment/
  obj.slots_for_sphere_dir = obj.CustomParametersDir + "/trajectory_planning/collision_environment/slots_for_sphere";
  assert(exist(obj.slots_for_sphere_dir)==7, "obj.slots_for_sphere_dir does not exist")
  disp(obj.slots_for_sphere_dir)
  [sphere_radius, sphere_origin_Z, Z_list, radius_list, adjusted_pts] = fit_sphere_to_shelves(obj); % Omit shelf_00 and shelf_12/shelf_12 since the cylinders cover them
  obj.radius_list = radius_list;
  obj.Z_list = Z_list;
  obj.shelf_pts = adjusted_pts;
  obj.sphere_buffer = 0.025;
  obj.sphere_radius = sphere_radius - obj.sphere_buffer; % introduce safety margin
  obj.sphere_origin = [0,0,sphere_origin_Z];
  obj.radius_offset = 0.005;  % Make path planning 0.005m more conservative, but do not use this with motioncheck.

  % Add cylinder on top/bottom to improve collision checking
  % (sphere cutoff + 2 cylinders);
  obj.cylinder_height = 0.25;
  obj.cylinder_buffer = 0.00;
  obj.sphere_cutoff_bottom = Z_list(1); % Switch to bottom cylinder for state space validator
  obj.sphere_cutoff_top = Z_list(end); % Switch to top cylinder for state space validator
  obj.bottom_cylinder_radius = obj.radius_list(1) - obj.cylinder_buffer; % Shelf_00 radius
  obj.top_cylinder_radius = obj.radius_list(end)- obj.sphere_buffer; % Larger buffer for top since there's more room (no pedestal)

 end

 end
end
