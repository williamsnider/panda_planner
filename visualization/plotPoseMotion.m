function finalAngles = plotPoseMotion(panda, poseArray, collisionObjectArray, initial_guess)
%PLOTJOINTMOTION Plots robot given angles
%   Detailed explanation goes here
% Display
controlHz = 1000;
dispHz = 15;



% Show panda first to get correct plot zoom
show(panda, initial_guess, 'Frames', 'off', 'Collisions', 'on');

% Get axis properties and set hold
ax = gca;
hold all;

% Show remaining objects
for i = 1:numel(collisionObjectArray)
    [~, patchObj] = show(collisionObjectArray{i}, "Parent", ax);
    patchObj.FaceColor = [0 1 1];
    patchObj.EdgeColor = 'none';
end



ik = inverseKinematics('RigidBodyTree', panda , 'SolverAlgorithm', 'LevenbergMarquardt');
weights = [1 1 1 1 1 1];

rateCtrlObj = rateControl(dispHz);
shortPoseArray = poseArray(:,:,1:round(controlHz/dispHz):end);
for i=1:size(shortPoseArray, 3)
    q = ik('panda_hand_tcp',shortPoseArray(:,:,i),weights,initial_guess);
    show(panda, q, "FastUpdate", true, "PreservePlot",false,"Visuals", "off", "Collisions", "on", 'Frames', 'off');
    initial_guess = q;
    if i == 1
        view(-134, 30)
    end
    waitfor(rateCtrlObj);
end

finalAngles = q;
end

