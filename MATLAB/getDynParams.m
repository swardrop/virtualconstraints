function [Is, ls, ms, g] = getDynParams()

% Define dynamical constants
m1 = 10;
l1 = 0.5;
r1 = 0.05;
I1 = m1*(1/4*r1^2 + 1/3*l1^2); % Moment of inertia about end of stance leg


m2 = 10;
l2 = 0.5;
r2 = 0.05;
I2 = 1/12*m2*(3*r2^2 + l2^2); % Moment of inertia about centre of swing leg


g = 9.81;

Is = [I1, I2];
ls = [l1, l2];
ms = [m1, m2];

end