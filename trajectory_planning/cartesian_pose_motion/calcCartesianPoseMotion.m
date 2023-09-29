function T = calcCartesianPoseMotion(startPose, goalPose, move_time, vMaxCartTrans, aMaxCartTrans, jMaxCartTrans, vMaxCartRot, aMaxCartRot, jMaxCartRot)



dist = norm(goalPose(1:3,4) - startPose(1:3,4));
s0 = 0;
[accelRiseTime,accelConstTime,accelZeroTime,peakAccel] = findOptimalProfileWithTimeConstraint(dist, s0, move_time/2, jMaxCartTrans, aMaxCartTrans, vMaxCartTrans);
total_length = accelRiseTime*2+accelConstTime+accelZeroTime;
assert(total_length <= move_time)
CONTROL_HZ = 1000;

% Calculate time constrained profile
timesteps = 0:1/CONTROL_HZ:2*total_length;  % Note: T is time to reach HALF of total distance

% Form this profile as piecewise polynomial
segTimes = [accelRiseTime accelConstTime accelRiseTime accelZeroTime accelZeroTime accelRiseTime accelConstTime accelRiseTime];
breaks = [0 cumsum(segTimes)];
s = sign(peakAccel);
m=s*jMaxCartTrans;
h=peakAccel;
coefs = [m 0; 0 h; -m h; 0 0; 0 0; -m 0; 0 -h; m -h];
ppA = mkpp(breaks, coefs);

% Get other profiles
ppJ = fnder(ppA);
ppV = fnint(ppA);
ppX = fnint(ppV, s0);  % Add in starting value for position
% plotProfiles(ppX, ppV, ppA, ppJ, vMaxCartTrans, aMaxCartTrans, jMaxCartTrans)

x = ppval(ppX,timesteps); % Note T = time to cover HALF of distance (symmetric)
y = x/x(end); % scale 0 to 1 for TimeScaling

TimeScaling = zeros(3,numel(timesteps));
TimeScaling(1,:) = y;

% Calculate 
T = transformtraj(startPose, goalPose, [timesteps(1), timesteps(end)], timesteps, 'TimeScaling', TimeScaling);

% Smooth + pad to ensure jerk is 0 at ends
window = 4;
T_extra = cat(3, repmat(T(:,:,1),1,1,window),T, repmat(T(:,:,end),1,1,window));
T_smooth = movmean(T_extra, window, 3, 'Endpoints','shrink');
% pad_size = 2
% T_pad = cat(3, repmat(T_smooth(:,:,1),1,1,pad_size), T_smooth,repmat(T_smooth(:,:,end),1,1,pad_size) );
% plot_derivatives(squeeze(T_pad(3,4,:)))
% 
% [x,dx,ddx,dddx] = calc_trajectory_derivatives(squeeze(T_smooth(3,4,:)));

T = T_smooth;
%% Check constraints met

% Translation
xyz = squeeze(T(1:3,4,:) - T(1:3,4,1));
dists = sqrt(sum(xyz.^2, 1))';
[c, dc, ddc, dddc] = calc_trajectory_derivatives(dists);
assert(max(abs(dc))-vMaxCartTrans<0.001)
assert(max(abs(ddc))-aMaxCartTrans<0.01)
assert(max(abs(dddc))-jMaxCartTrans<0.01)

% Rotation
angles = zeros(size(T,3),1);
for i = 1:size(T,3)
    
    % Calculate difference of rotation
    R = T(1:3,1:3,1) * T(1:3,1:3,i)';

    % Calculate angle
    angles(i) = acos( (trace(R) - 1) / 2);
end

[a, da, dda, ddda] = calc_trajectory_derivatives(angles);
assert(max(abs(da))-vMaxCartRot<0.001)
assert(max(abs(dda))-aMaxCartRot<0.01)
assert(max(abs(ddda))-jMaxCartRot<0.01)

end
