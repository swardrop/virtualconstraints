function [Gamma, Psi, th_base, th_c, alpha, beta, gamma, ...
    Phi, dPhi, ddPhi] = PartialSolZeroDyn(theta_p, alpha_p, num_points)
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
nck = precalcnck(length(theta_p)-1);

[alpha, beta, gamma, Phi, dPhi, ddPhi, al_zc] = ...
    interval(th_base, theta_p, alpha_p, nck);

% Identify critical theta (gamma zero crossing)
if (sign(gamma(1)) ~= sign(gamma(end))) && ~any(isnan(gamma))
    th_c = fzero(@(t)gam2(t,theta_p,alpha_p, nck),[theta_0, theta_f]);
else
    th_c = inf; % No critical th_c.
end
al_zc = [];
% Partition theta_0 - theta_f wherever there is a zero crossing, and
% resample such that the sampling around the zero crossing is symmetrical
for i = 1 : length(al_zc)
    % Find the lesser of halfway to the next or previous zc or to the
    % start or end of the interval. Note that al_zc is on the left of
    % the zero crossing.
    if i == 1
        nump = min([al_zc(i)-1, ...
            floor(0.5*(al_zc(i+1)-(al_zc(i)+1)))+1]);
    elseif i == length(al_zc)
        nump = min([floor(0.5*(al_zc(i)-(al_zc(i-1)+1))), ...
            num_points-(al_zc(i)+1)]);
    else
        nump = min([floor(0.5*(al_zc(i+1)-(al_zc(i)+1)))+1, ...
            floor(0.5*(al_zc(i)-(al_zc(i-1)+1)))]);
    end
    % Find the point of zero crossing
    th_zc = fzero(@(t)al2(t,theta_p, alpha_p, nck), ...
        [th_base(al_zc(i)), th_base(al_zc(i)+1)]);
    % Construct intervals sampled symmetrically about the zero crossing
    i1 = al_zc(i)-nump; i2 = al_zc(i); 
    i3 = al_zc(i)+1; i4 = al_zc(i)+1+nump;
    th_base(i1:i2) = linspace(th_zc-nump*step_size, th_zc-eps, i2-i1+1);
    th_base(i3:i4) = linspace(th_zc+eps, th_zc+nump*step_size, i4-i3+1);
    [alpha(i1:i4), beta(i1:i4), gamma(i1:i4), ...
        Phi(:,i1:i4), dPhi(:,i1:i4), ddPhi(:,i1:i4)] = ...
        interval(th_base(i1:i4), theta_p, alpha_p, nck);
end

fx = 2*beta./alpha;
gx = -2*gamma./alpha;

int_fx = zeros(size(th_base));
int_gxefx = zeros(size(th_base));
ind = 1;
al_zc(end+1) = length(th_base); % Slight abuse of notation here
cum_fx = 0; cum_gx = 0;
for i = 1 : length(al_zc)
    int_fx(ind:al_zc(i)) = cum_fx + cumtrapz(th_base(ind:al_zc(i)), ...
        fx(ind:al_zc(i)), 2);
    int_gxefx(ind:al_zc(i)) = cum_gx + cumtrapz(th_base(ind:al_zc(i)), ...
        gx(ind:al_zc(i)).* exp(int_fx(ind:al_zc(i))), 2);
    ind = al_zc(i)+1;
    cum_fx = int_fx(al_zc(i));
    cum_gx = int_gxefx(al_zc(i));
end


Gamma = exp(-int_fx);
Psi = exp(-int_fx).*int_gxefx;
end





function [alpha, beta, gamma, Phi, dPhi, ddPhi, al_zc] = ...
    interval(th_base, theta_p, alpha_p,nck)

alpha = zeros(size(th_base));
beta = zeros(size(th_base));
gamma = zeros(size(th_base));
Phi = zeros([size(alpha_p,1)+1, length(th_base)]);
dPhi = zeros(size(Phi));
ddPhi = zeros(size(Phi));
al_zc = [];

for i = 1 : length(th_base)
    % Calculate zero dynamics coefficients
    [Phi(:,i), dPhi(:,i), ddPhi(:,i)] = bezier(theta_p, alpha_p, ...
        th_base(i),nck);
    [M, C, G, ~, B_perp] = dynMatrices(Phi(:,i), dPhi(:,i));
    alpha(i) = al(B_perp, M, dPhi(:,i));
    beta(i) = bet(B_perp, M, C, dPhi(:,i), ddPhi(:,i));
    gamma(i) = gam(B_perp, G);
    
    % Test for zero crossings of alpha
    if i>1 && sign(alpha(i-1)) ~= sign(alpha(i))
        al_zc(end+1) = i-1; %#ok<AGROW>
    end
end

end

% Precalculate the values of Nchoosek
function ncks = precalcnck(N)
for i = N+1:-1:1
    ncks(i) = nchoosek(N,i-1);
end
end

function alpha = al(B_perp, M, dPhi)
alpha = B_perp*M*dPhi;
end

function beta = bet(B_perp, M, C, dPhi, ddPhi)
beta = B_perp*(M*ddPhi + C*dPhi);
end

function gamma = gam(B_perp, G)
gamma = B_perp*G;
end

function alpha = al2(theta, theta_p, alpha_p, nck)
[Phi, dPhi] = bezier(theta_p, alpha_p, theta, nck);
[M,~,~,~,B_perp] = dynMatrices(Phi, dPhi);
alpha = B_perp*M*dPhi;
end

function gamma = gam2(theta, theta_p, alpha_p, nck)
[Phi, dPhi] = bezier(theta_p, alpha_p, theta, nck);
[~,~,G,~,B_perp] = dynMatrices(Phi, dPhi);
gamma = B_perp*G;
end