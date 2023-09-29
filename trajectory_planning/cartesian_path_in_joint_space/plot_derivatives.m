function plot_derivatives(x, params)
% Estimate derivatives
figure
[x, dx, ddx, dddx] = calc_trajectory_derivatives(x);

ax = subplot(1,5,1);
scaled = (x-params.jointMin)./(params.jointMax-params.jointMin);
plot(scaled)
title('scaled')
legend()

ax = subplot(1,5,2);
plot(x)
title('x')
legend()

ax = subplot(1,5,3);
plot(dx)
title('dx')

ax = subplot(1,5,4);
plot(ddx)
title('ddx')

ax = subplot(1,5,5);
plot(dddx)
title('dddx')
end

