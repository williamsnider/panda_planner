function [x,y,z,h,success] = caseB(A, T, m, aMax, vMax)

% Case 2 - Reach max acceleration (h=aMax)
h=aMax;

% A = 3hyx/2 + hy^2/2 + hx^2 + zhy + zhx T = x+y+z+x x = h/m
coeffs = [-h/2, -3*h^2/(2*m)+h*T, (-A-h^3/(m^2) + h^2*T/m)];
y_roots = roots(coeffs);
y_real = y_roots(imag(y_roots)==0);

success = false;
for i=1:numel(y_real)

    y = y_real(i);

    % Derived results
    x = h/m;
    z = T-2*x-y;

    % Round z to 0 for very small negative Z's
    if sign(z) == -1 && abs(z) < eps(1e9)
        z = 0;
        y=T-2*x;
    end

    % Check valid profile (constriants met)
    valid = checkProfile(x,y,z,h,A, vMax, aMax);
    
    if valid
        success = true;
        break
    end
end

assert(success)


end
