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
 checkSteps = 100  % ms; every 250ms is checked by motionCheck for environmental collisions
 checkProportion = 0.05 % Proportion of states that are double checked for kinematic constraints and self/environment collisiosn
 smoothing_window_size = 100 % Smooths out joint-to-joint motions
    multiMovementPadding = 50  % For a trajectory with multiple steps, pad the start/end of each step by this amount to smooth the transitions
 
    % Joint positions
 q_home = [-0.0121707, -0.561084, 0.00127942, -2.60702, -0.0211893, 2.03285, 0.802306, 0.01, 0.01];
 
 
 % Model held shapes as cylinders
 cylinderRadius = 0.0225;  
 cylinderLength = 0.125; 
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
staging_box_width = 0.37; 
staging_box_height = 1.2; 
staging_box_length = 0.85; 
staging_box_center = [-0.60, -0.05, 0.5]; % Replace with the desired center coordinates [x, y, z]
            


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
