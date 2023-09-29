clear; close all;
addpath("..")

numJoints = 7;
i=0;
while true
    %     if mod(i,10000) == 0
    %         disp(i)
    %     end
    disp(i)
%         rng(i)
    i=i+1;
    
    s0All = zeros(numJoints,1);
    s1All = zeros(numJoints,1);
    vMaxAll = zeros(numJoints,1);
    aMaxAll = zeros(numJoints,1);
    jMaxAll = zeros(numJoints,1);
    
    for j=1:numJoints
       
        % Constraints
        aMax = rand(1)*1e1+eps;
        vMax = rand(1)*1e1+eps;
        jMax = rand(1)*1e1+eps;
        
        % Inputs
        s1 = rand(1)*1e1;
        s0 = rand(1)*1e1;
        if (s1-s0==0)
            continue
        end
        
        s0All(j) = s0;
        s1All(j) = s1;
        vMaxAll(j) = vMax;
        aMaxAll(j) = aMax;
        jMaxAll(j) = jMax;
    end
    
    
    % Find optimal profile
    allJointTrajectories = findMultipleJointProfiles(s0All, s1All, vMaxAll, aMaxAll, jMaxAll);
    
    % Handle case where too many timesteps requested for evaluation
    % (otherwise crashes)
    if isempty(allJointTrajectories)
        continue
    else
        valid = checkTrajectory(allJointTrajectories,s0All, s1All, vMaxAll, aMaxAll, jMaxAll);
    end
    
end
    