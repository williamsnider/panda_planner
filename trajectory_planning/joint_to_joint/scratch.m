format long

addpath([pwd, '/../..'])
run parameters.m

r = 0.75;
x0 =  0;
y0 =  0;
z0 =  0.391;

%% Get sample path
% Inputs
start = [-0.0121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.05, 0.00];
goal = [-0.1121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.01,0.01];

% Load robot
addpath([pwd, '/../robot'])
panda = loadPandaWithShape();

% Load collision environment
addpath([pwd, '/../collision_environment'])
env = build_collision_environment();
env = env(440:end);

% Plan
all_trajectory = planJointToJoint(panda, env, start,goal, vMaxAll, aMaxAll, jMaxAll);


%% Find robot points along trajectory
q = start;

filename = panda.Bodies{5}.Collisions{1};
filename = erase(filename, 'Mesh Filename ');
% TR = stlread(filename); % contains "Mesh" structure

%% Import robot
urdf = strcat(pwd,'/panda_description/panda.urdf')
rob = importrobot(urdf, 'DataFormat', 'row')

% Base
name = 'link0';
filename = strcat(pwd, '/panda_description/visual/', name, '.dae');
addVisual(rob.Base, 'Mesh',  filename)
filename = strcat(pwd, '/panda_description/collision/', name, '.stl');
addCollision(rob.Base, 'Mesh',  filename)

for i=1:11

    if i<=7
        name = strcat('link',num2str(i));
    elseif i==8
        continue
    elseif i==9
        name='hand';
    else
        name='finger';
    end

    % Add visual mesh
    filename = strcat(pwd, '/panda_description/visual/', name, '.dae');
    addVisual(rob.Bodies{i}, 'Mesh',  filename)

    % Add collision mesh
    if strcmp(name, "finger")
        addCollision(rob.Bodies{i}, 'cylinder',  [0.1, 0.1])
    else
        filename = strcat(pwd, '/panda_description/collision/', name, '.stl');
        addCollision(rob.Bodies{i}, 'Mesh',  filename)
    end
end

%TODO: Figure out why finger visuals are weird, located in wrong frame?


show(rob, start, 'Visuals', 'on', 'Collisions', 'on')