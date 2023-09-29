function plotJointScaled(q,qMax, qMin)
figure

ax = subplot(1,1,1);
q_scaled = (q-qMin)./(qMax-qMin);
plot(q_scaled)
title('q_scaled')
ylim([-0.05, 1.05])

legend()
end
