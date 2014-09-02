% Produces the partial closed-form solution for the square of the velocity
% of the phase variable theta in terms of the two coefficient functions,
% Gamma and Psi, where
% theta_dot^2 = Gamma*theta_dot_0^2 + Psi;
% Gamma and Psi both being functions of theta,
% when the physical system is subject to the holonomic constraint defined
% by the Bezier control points defined by the matrix constrPts.

function [Gamma, Psi, th_base, th_c] = PartialSolZeroDyn(constrPts)

theta_f = constrPts(end,1);
theta_0 = constrPts(1,1);
num_points = 500;
th_base = linspace(theta_0,theta_f,num_points);
step_size = (theta_f-theta_0)/num_points;

alpha = zeros(size(th_base));
beta = zeros(size(th_base));
gamma = zeros(size(th_base));
th_c = inf;

for i = 1 : num_points
    % Calculate zero dynamics coefficients
    [al, bet, gam] = ZeroDynCompassGait(constrPts, th_base(i));
    alpha(i) = al;
    beta(i) = bet;
    gamma(i) = gam;
    
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