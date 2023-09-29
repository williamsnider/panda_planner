function [q, dq, ddq, dddq] = calc_trajectory_derivatives(q)
% q must have shape numTimeSteps x numJoints 
numJoints = size(q, 2);
dq = [zeros(1, numJoints); diff(q, 1, 1)/0.001];
ddq = [zeros(1, numJoints); diff(dq, 1, 1)/0.001];
dddq = [zeros(1, numJoints); diff(ddq, 1, 1)/0.001];
end
