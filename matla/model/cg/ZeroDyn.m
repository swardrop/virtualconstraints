function [alpha, beta, gamma, Phi, d_Phi, dd_Phi] = ...
    ZeroDynCompassGait(theta_p, alpha_p, theta)
% Produces the zero dynamics of the Compass-gait walker with dynamical
% parameters defined by dynMatrices(q,qd) subject to the Bezier control 
% points in {theta_p, alpha_p} at theta.

theta_f = theta_p(end);
theta_0 = theta_p(1);
n = length(theta_p) - 1;

% Produce angle phi from theta
s = (theta - theta_0) / (theta_f - theta_0);
s(s>1) = 1;
s(s<0) = 0;
phi = 0;
for i = 0 : n
    phi = phi + nchoosek(n,i) * (1-s)^(n-i) * s^i * alpha_p(:,i+1);
end
% Derivative of phi
d_phi = n*(s^(n-1)*alpha_p(:,n+1) - (1-s)^(n-1)*alpha_p(:,1));
for i = 1 : n-1
    d_phi = d_phi + ...
        nchoosek(n,i)*(i-n*s)*s^(i-1)*(1-s)^(n-i-1)*alpha_p(:,i+1);
end
d_phi = d_phi / (theta_f-theta_0);
% 2nd derivative of phi
dd_phi = (n-1)*n*((1-s)^(n-2)*alpha_p(:,1) + s^(n-2)*alpha_p(:,n+1)) ...
    + n*((n-1)*(n-2)*s*(1-s)^(n-3) - 2*(n-1)*(1-s)^(n-2))*alpha_p(:,2) ...
    + n*((n-1)*(n-2)*s^(n-3)*(1-s) - 2*(n-1)*s^(n-2))*alpha_p(:,n);
for i = 2 : n-2
    dd_phi = dd_phi + nchoosek(n,i)*s^(i-2)*(1-s)^(n-i-2)* ...
        (i^2 - i*(2*(n-1)*s + 1) + n*(n-1)*s^2)*alpha_p(:,i+1);
end
dd_phi = dd_phi / (theta_f-theta_0)^2;

%%% Prepare matrices
% Variables
Phi = [phi; theta];
d_Phi = [d_phi; 1];
dd_Phi = [dd_phi; 0];

[M, C, G, ~, B_perp] = dynMatrices(Phi, d_Phi);

alpha = B_perp*M*d_Phi;
beta = B_perp*(M*dd_Phi + C*d_Phi);
gamma = B_perp*G;