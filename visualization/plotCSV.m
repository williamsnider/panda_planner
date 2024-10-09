function plotCSV(panda_sc, filename, env, params)

% Load csv
anglesArray = readmatrix(filename);

% Add in finger positions
fingerPositions = 0.01*ones([size(anglesArray,1),2]);
anglesArray = [anglesArray, fingerPositions];

% Plot motion
plotJointMotion(panda_sc, anglesArray, env, params)
end