function [alpha, beta, gamma, Phi, d_Phi, dd_Phi] = ...
    ZeroDyn(theta_p, alpha_p, theta)
% Produces the zero dynamics of the robot with dynamical
% parameters defined by dynMatrices(q,qd) subject to the Bezier control 
% points in {theta_p, alpha_p} at theta.

[phi, d_phi, dd_phi] = bezier(theta_p, alpha_p, theta);

Phi = [phi; theta];
d_Phi = [d_phi; 1];
dd_Phi = [dd_phi; 0];

[M, C, G, ~, B_perp] = dynMatrices(Phi, d_Phi);

alpha = B_perp*M*d_Phi;
beta = B_perp*(M*dd_Phi + C*d_Phi);
gamma = B_perp*G;