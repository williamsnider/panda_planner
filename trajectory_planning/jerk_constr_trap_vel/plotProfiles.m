function plotProfiles(ppX, ppV, ppA, ppJ, vMax, aMax, jMax)

breaks = ppX.breaks;

% Plot pp
subplot(4,1,1)
data = ppX;
xq = breaks(1):0.0001:breaks(end);
plot(xq,ppval(data,xq))
for i = 1:numel(breaks)
    xline(breaks(i),'--k')
end
xlim([breaks(1), breaks(end)])
title('ppX')

% Plot pp
subplot(4,1,2)
data = ppV;
xq = breaks(1):0.0001:breaks(end);
plot(xq,ppval(data,xq))
for i = 1:numel(breaks)
    xline(breaks(i),'--k')
end
xlim([breaks(1), breaks(end)])
yline(vMax, '--r')
ylim(sort([-1.1*vMax, 1.1*vMax]))
title('ppV')

% Plot pp
subplot(4,1,3)
data = ppA;
xq = breaks(1):0.0001:breaks(end);
plot(xq,ppval(data,xq))
for i = 1:numel(breaks)
    xline(breaks(i),'--k')
end
xlim([breaks(1), breaks(end)])
ylim(sort([-1.1*aMax, 1.1*aMax]))
yline(-aMax, '--r')
yline(-aMax, '--r')
title('ppA')

% Plot pp
subplot(4,1,4)
data = ppJ;
xq = breaks(1):0.0001:breaks(end);
plot(xq,ppval(data,xq))
for i = 1:numel(breaks)
    xline(breaks(i),'--k')
end
xlim([breaks(1), breaks(end)])
ylim(sort([-1.1*jMax, 1.1*jMax]))
yline(jMax, '--r')
yline(-jMax, '--r')
title('ppJ')