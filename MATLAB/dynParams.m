function [I, l, m, g] = dynParams()

% Define dynamical constants
m = 0.5;
l = 1;
r = 0.05;
I = 1/12*m*(3*r^2 + l^2); % Moment of inertia about centre of leg
g = 9.81;

end