clear; close all;
addpath("..")

numJoints = 7;

s0All = zeros(numJoints,1);
s1All = zeros(numJoints,1);
vMaxAll = zeros(numJoints,1);
aMaxAll = zeros(numJoints,1);
jMaxAll = zeros(numJoints,1);

% Require motion in just 1 joint
j = 3;
s0All(j) = -0.2;
s1All(j) = -0.25;
vMaxAll(j) = 0.2;
aMaxAll(j) = 0.2;
jMaxAll(j) = 0.2;

allJointTrajectories = findMultipleJointProfiles(s0All, s1All, vMaxAll, aMaxAll, jMaxAll);
   
