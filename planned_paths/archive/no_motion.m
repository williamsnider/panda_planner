% Generate csv of joint positions being held at a starting pose
format long

% Inputs
num_steps = 2000;
q = [-0.0121707,-0.561084,0.00127942,-2.60702,-0.0211893,2.03285,0.802306, 0.01, 0.01];

% Generate postisions
all_trajectory = repmat(q', 1, num_steps);

% Export as csv
writematrix(all_trajectory(1:7, :)', 'no_motion.csv')

