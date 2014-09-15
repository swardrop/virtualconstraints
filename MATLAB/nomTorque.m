function [u, theta, G_c, P_c] = nomTorque(constrPts, theta_dot_sq_0)
%NOMTORQUE Calculates the nominal torque required to maintain zero
% dynamics of the compass-gait walker given initial theta-dot^2
%   Output is a grid (for now) of theta values with corresponding torques.
%   This should ideally be replaced by some kind of polynomial.

[I, l, m, g] = getDynParams;
I2 = I(2);
l = l(1);
m = m(1);

[Gamma, Psi, theta, th_c, alpha, beta, gamma, phi, d_phi, dd_phi] = ...
    PartialSolZeroDyn(constrPts);

th_d_sq = Gamma*theta_dot_sq_0 + Psi;
th_dd = -beta./alpha.*th_d_sq - gamma./alpha;

t2dd = dd_phi.*th_d_sq + d_phi.*th_dd;

u = t2dd*(I2 + 0.25*m*l^2) + ...
    th_dd.*(I2 + 0.5*m*(0.5*l^2 + l^2*cos(phi))) ...
    + th_d_sq.*(0.5*m*l^2*sin(phi)) + 0.5*m*g*l*cos(theta+phi);

% Calculate Gamma and Psi at th_c.
step_size = theta(2)-theta(1);
idx_before = floor((th_c-theta(1))/step_size) + 1; % 1-indexing
G_c = (Gamma(idx_before+1)-Gamma(idx_before))/step_size * ...
    (th_c - theta(idx_before)) + Gamma(idx_before);
P_c = (Psi(idx_before+1)-Psi(idx_before))/step_size * ...
    (th_c - theta(idx_before)) + Psi(idx_before);
end