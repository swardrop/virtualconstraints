function [Phi, dPhi, ddPhi] = bezier(theta_p, alpha_p, theta, nck)

theta_0 = theta_p(1);
theta_f = theta_p(end);
% Produce angle phi from theta
N = length(theta_p) - 1;
s = (theta - theta_0) / (theta_f - theta_0);
s(s>1) = 1;
s(s<0) = 0;
phi = 0;
for i = 0 : N
    phi = phi + nck(i+1) * (1-s)^(N-i) * s^i * alpha_p(:,i+1);
end
% Derivative of phi
d_phi = N*(s^(N-1)*alpha_p(:,N+1) - (1-s)^(N-1)*alpha_p(:,1));
for i = 1 : N-1
    d_phi = d_phi + ...
        nck(i+1)*(i-N*s)*s^(i-1)*(1-s)^(N-i-1)*alpha_p(:,i+1);
end
d_phi = d_phi / (theta_f-theta_0);
% 2nd derivative of phi
dd_phi = (N-1)*N*((1-s)^(N-2)*alpha_p(:,1) + s^(N-2)*alpha_p(:,N+1)) ...
    + N*((N-1)*(N-2)*s*(1-s)^(N-3) - 2*(N-1)*(1-s)^(N-2))*alpha_p(:,2) ...
    + N*((N-1)*(N-2)*s^(N-3)*(1-s) - 2*(N-1)*s^(N-2))*alpha_p(:,N);
for i = 2 : N-2
    dd_phi = dd_phi + nck(i+1)*s^(i-2)*(1-s)^(N-i-2)* ...
        (i^2 - i*(2*(N-1)*s + 1) + N*(N-1)*s^2)*alpha_p(:,i+1);
end
dd_phi = dd_phi / (theta_f-theta_0)^2;

Phi = [phi; theta];
dPhi = [d_phi; 1];
ddPhi = [dd_phi; 0];
