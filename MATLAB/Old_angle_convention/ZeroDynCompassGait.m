% Produces the zero dynamics of the Compass-gait walker with dynamical
% parameters defined by the getDynParams() function subject to the Bezier
% control points in constrPts, at the point through the constraint defined
% by theta.

function [alpha, beta, gamma, phi, d_phi, dd_phi] = ...
    ZeroDynCompassGait(constrPts, theta)

[Is, ls, ms, g] = getDynParams();
I1 = Is(1);
I2 = Is(2);
l1 = ls(1);
l2 = ls(2);
m1 = ms(1);
m2 = ms(2);
theta_f = constrPts(end,1);
theta_0 = constrPts(1,1);

% Produce angle phi from theta
phi = 0;
n = size(constrPts,1) - 1;
for i = 0 : n
    phi = phi + nchoosek(n,i)*(theta_f-theta)^(n-i)*...
        (theta-theta_0)^i*constrPts(i+1,2);
end
phi = phi/(theta_f - theta_0)^n;

% Calculate derivative of phi
d_phi = n*((theta-theta_0)^(n-1)*constrPts(n+1,2) ...
    - (theta_f-theta)^(n-1)*constrPts(1,2));
for i = 1 : n-1
    d_phi = d_phi + nchoosek(n,i)*constrPts(i+1,2) * (...
        i*(theta_f-theta)^(n-i)*(theta-theta_0)^(i-1) - ...
        (n-i)*(theta_f-theta)^(n-i-1)*(theta-theta_0)^i);
end
d_phi = d_phi/(theta_f - theta_0)^n;
% Calculate second derivative of phi
dd_phi = n*(n-1)*((theta_f-theta)^(n-2)*constrPts(1,2) + ...
    (theta-theta_0)^(n-2)*constrPts(n+1,2) + (n-2)*(...
    (theta-theta_0)*(theta_f-theta)^(n-3)*constrPts(2,2) + ...
    (theta_f-theta)*(theta-theta_0)^(n-3)*constrPts(n,2)) - 2*(...
    (theta_f-theta)^(n-2)*constrPts(2,2) + ...
    (theta-theta_0)^(n-2)*constrPts(n,2)));
for i = 2 : n-2
    dd_phi = dd_phi + nchoosek(n,i)*constrPts(i+1,2) * (...
        i*(i-1)*(theta_f-theta)^(n-i)*(theta-theta_0)^(i-2) - ...
        2*i*(n-i)*(theta_f-theta)^(n-i-1)*(theta-theta_0)^(i-1) + ...
        (n-i-1)*(n-i)*(theta_f-theta)^(n-i-2)*(theta-theta_0)^i );
end
dd_phi = dd_phi/(theta_f - theta_0)^n;

%%% Prepare matrices
% Variables
d_Phi = [1; d_phi];
dd_Phi = [0; dd_phi];
% Compute mass/inertia matrix
M = [I1+I2+m2*(l1^2+0.25*l2^2+l1*l2*cos(phi)), ...
    I2+0.5*m2*(0.5*l2^2+l1*l2*cos(phi)); ...
    I2+0.5*m2*(0.5*l2^2+l1*l2*cos(phi)), ...
    I2+0.25*m2*l2^2];
% Compute coriolis/centrifugal matrix
C = sin(phi)*[-m2*l1*l2*d_phi, -0.5*m2*l1*l2*d_phi; 0.5*m2*l1*l2, 0];
% Compute gradient of gravitational potential energy
G = [0.5*l1*m1*g*cos(theta) + m2*g*(l1*cos(theta)+0.5*l2*cos(theta+phi))
    0.5*m2*g*l2*cos(theta+phi)];
% Use trivial B_perp
B_perp = [1 0];

alpha = B_perp*M*d_Phi;
beta = B_perp*(M*dd_Phi + C*d_Phi);
gamma = B_perp*G;