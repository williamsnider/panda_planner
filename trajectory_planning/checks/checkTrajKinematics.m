function valid = checkTrajKinematics(allJointTrajectories,s0All, s1All, params)
%CHECKTRAJECTORY Checks if a joint trajectory respects position, velocity,
%acceleration, and jerk constraints.

sMaxAll = params.sMaxAllAbsolute;
sMinAll = params.sMinAllAbsolute;
vMaxAll = params.vMaxAll;
aMaxAll = params.aMaxAll;
jMaxAll = params.jMaxAll;

assert (size(allJointTrajectories,2) == size(s0All,2))

numJoints = size(s0All, 2);

% Estimate derivatives
[~, dx, ddx, dddx] = calc_trajectory_derivatives(allJointTrajectories);

% Get peaks
vPeak = max(abs(dx),[],1);
aPeak = max(abs(ddx),[],1);
jPeak = max(abs(dddx),[],1);

%% Checks
valid = true;

% Starting position
diffStart = abs(allJointTrajectories(1, :) - s0All);
validStart = all(diffStart < eps(1e5));
if ~validStart
    disp('Starting position not matching start of joint trajectory.')
    valid=false;
end


% Final position
diffFinal = abs(allJointTrajectories(end,:) - s1All);
validFinal = all(diffFinal < 0.0000001);
if ~validFinal
    disp('Final desired position not matching end of joint trajectory.')
    valid=false;
end

% Positions
if any(allJointTrajectories - sMaxAll > 0.001)
    disp('Position exceeding maximum.')
    valid=false;
end
if any(allJointTrajectories - sMinAll < -0.001)
    disp('Position exceeding minimum.')
    valid=false;
end

% Velocity
validVel = all(vMaxAll - vPeak > -0.1);
if ~validVel
    disp('Velocity exceeding scaled maximum.')
    valid=false;
end

% Acceleration
validAccel = all(aMaxAll - aPeak > -1.0);
if ~validAccel
    disp('Acceleration exceeding scaled maximum.')
    valid=false;
end

% Jerk
validJerk = all(jMaxAll - jPeak > -1.0);
validJerkStart = all(dddx(1,:)==0);  % Jerk 0 at start
validJerkEnd = all(abs(dddx(end,:))==0);  % Jerk 0 at end
if any([~validJerk, ~validJerkStart, ~validJerkEnd])
    disp('Jerk not valid, either exceding maximum or not 0 at start/end.')
    valid=false;
end

% if ~valid
%     zzz = 0;
% end
% %% Print Summary
% 
% disp(strcat(num2str(size(allJointTrajectories, 1)/1000), "s total duration"))
% 
% vPercentage = round(vPeak./vMaxAll*100);
% aPercentage = round(aPeak./aMaxAll*100);
% jPercentage = round(jPeak./jMaxAll*100);
% arr = [vPercentage', aPercentage', jPercentage'];
% colNames = {'vPeak', 'aPeak', 'jPeak'};
% rowNames = {'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7', 'g8', 'g9'};
% tab = array2table(arr, 'RowNames', rowNames, 'VariableNames', colNames);
% disp(tab)
end

