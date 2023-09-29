function scaled = calc_scaled(q, params)
scaled = (q(1:7)-params.jointMin)./(params.jointMax-params.jointMin);
end

