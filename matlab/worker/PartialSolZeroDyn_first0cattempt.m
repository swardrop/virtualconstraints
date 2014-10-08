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
eps = 1e-8;

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
al_0c = [];

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
        alpha_prev = al;
    end
    if sign(gamma_prev) ~= sign(gam)
        % Use linear interpolation between two points
        th_c = gamma_prev/(gamma_prev-gam)*step_size+th_base(i)-step_size;
    end
    gamma_prev = gam;
    
    % Test for alpha zero crossing
    if sign(alpha_prev) ~= sign(al)
        al_0c(end+1) = i-1; %#ok<AGROW>
    end
    alpha_prev = al;
end

fx = 2*beta./alpha;
gx = -2*gamma./alpha;

% Partition fx and gx such that any intervals where there are zero
% crossings are excluded from the integral
idx = 1;
int_fx = zeros(size(th_base));
int_gxefx = zeros(size(th_base));

for i = 1 : length(al_0c)
    zc = fzero(@(theta)ZeroDyn(theta_p,alpha_p,theta), ...
        [th_base(al_0c(i)), th_base(al_0c(i)+1)]);
    [alm, betm, gamm] = ZeroDyn(theta_p, alpha_p,zc-eps);
    fx(al_0c(i)) = 2*betm/alm;
    gx(al_0c(i)) = -2*gamm/alm;
    th_base(al_0c(i)) = zc-eps;
    [alp, betp, gamp] = ZeroDyn(theta_p, alpha_p,zc+eps);
    fx(al_0c(i)+1) = 2*betp/alp;
    gx(al_0c(i)+1) = -2*gamp/alp;
    th_base(al_0c(i)+1) = zc+eps;
    int_fx(idx:al_0c(i)) = cumtrapz(fx(idx:al_0c(i)), ...
        th_base(idx:al_0c(i)));
    int_gxefx(idx:al_0c(i)) = cumtrapz(gx(idx:al_0c(i)) ... 
        .*exp(int_fx(idx:al_0c(i))), th_base(idx:al_0c(i)));
    idx = al_0c + 1;
end

int_fx(idx:end) = cumtrapz(fx(idx:end), th_base(idx:end));
int_gxefx(idx:end) = cumtrapz(gx(idx:end).*exp(int_fx(idx:end)), ...
    th_base(idx:end));
Gamma = exp(-int_fx);
Psi = exp(-int_fx).*int_gxefx;
end