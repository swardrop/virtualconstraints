function [M, C, G, B, B_perp] = dynMatrices(q,qd)
% Matrices which define the dynamics of the Compass-Gait walker:
% M(q)qdd + C(q,qd)qd + G(q) = B(q)u

[I, l, m, g] = dynParams();

M = [0.25*m*l^2 + I, ...
    0.5*m*l^2*(cos(q(1)) - 1/2) - I; ...
    0.5*m*l^2*(cos(q(1)) - 1/2) - I, ...
    m*l^2*(1.5 - cos(q(1))) + 2*I];

C = 0.5*m*l^2*sin(q(1))*[0, -qd(2); qd(2)-qd(1), qd(1)];

G = 0.5*m*g*l*[sin(q(1)-q(2)); -sin(q(1)-q(2))-3*sin(q(2))];

B = [1;0];

B_perp = [0 1]; % Simple non-zero matrix s.t. B_perp*B = 0

end