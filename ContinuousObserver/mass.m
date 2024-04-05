function M = mass(q)

% Calculate the mass matrix

% cos and sine
c2 = cos(q(2));
c3 = cos(q(3));
c23 = cos(q(2)+q(3));

% M-components
m11 = 1.0425+0.08094*c23+0.3484*c2+.0561*c3;
m12 = 0.4398+.04047*c23+.1742*c2+.0561*c3;
m13 = .1788+.04047*c23+.02809*c3;
m22 = .4398+.0519*c3;
m23 = .1788+.02809*c3;
m33 = .1788;
% Mass-Inertia matrix
M = [m11 m12 m13;
     m12 m22 m23;
     m13 m23 m33];


end

