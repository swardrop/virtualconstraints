function [Gamma, Psi, th_base, th_c, alpha, beta, gamma, ...
    Phi, d_Phi, dd_Phi] = PartialSolZeroDyn(theta_p, alpha_p, num_points)
% Produces the partial closed-form solution for the square of the velocity
% of the phase variable theta in terms of the two coefficient functions,
% Gamma and Psi, where
% theta_dot^2 = Gamma*theta_dot_0^2 + Psi;
% Gamma and Psi both being functions of theta,
% when the physical system is subject to the holonomic constraint defined
% by the Bezier control points defined by theta_p and alpha_p.

if nargin < 3
    num_points = 25;
end

theta_f = theta_p(end);
theta_0 = theta_p(1);
th_base = linspace(theta_0,theta_f,num_points);
step_size = (theta_f-theta_0)/num_points;

alpha = zeros(size(th_base));
beta = zeros(size(th_base));
gamma = zeros(size(th_base));
Phi = zeros([size(alpha_p,1)+1, length(th_base)]);
d_Phi = zeros(size(Phi));
dd_Phi = zeros(size(Phi));
th_c = inf;

for i = 1 : num_points
    % Calculate zero dynamics coefficients
    [al, bet, gam, P, dP, ddP] = ...
        ZeroDyn(theta_p, alpha_p, th_base(i));
    alpha(i) = al;
    beta(i) = bet;
    gamma(i) = gam;
    Phi(:,i) = P;
    d_Phi(:,i) = dP;
    dd_Phi(:,i) = ddP;
    
    % Find th_c
    if i == 1
        gamma_prev = gam;
    end
    if (gam < 0 && gamma_prev > 0) || (gam > 0 && gamma_prev < 0)
        % Use linear interpolation between two points
        th_c = gamma_prev/(gamma_prev-gam)*step_size+th_base(i)-step_size;
    end
    gamma_prev = gam;
end

fx = 2*beta./alpha;
gx = -2*gamma./alpha;
    
int_fx = cumtrapz(fx)*step_size;
int_gxefx = cumtrapz(gx.*exp(int_fx))*step_size;
Gamma = exp(-int_fx);
Psi = exp(-int_fx).*int_gxefx;

end