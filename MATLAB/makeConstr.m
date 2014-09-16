function [constrData, Gamma, Psi, th_base] = makeConstr(theta_p, alpha_p)
% Takes a set of Bezier control points which are specified in each row of
% the points argument. Returns a structure containing the following fields:
% step_l - length of step
% step_h - height of step
% Gamma_c - value of Gamma function at critical theta
% Psi_c - value of Psi function at critical theta
% theta_c - critical theta
% Gamma_f - value of Gamma function just before impact
% Psi_f - value of Psi function just before impact
% alpha_p - Bezier coefficients (identical to supplied points)
% theta_p - The start and end points of the phase variable

% Calculate step length and height
[s_l, s_h] = endSwingFoot(bezConstraint(theta_p, alpha_p, theta_p(end)));

% Calculate partial solution
[Gamma, Psi, th_base, th_c] = PartialSolZeroDyn(theta_p, alpha_p);

% Calculate Gamma and Psi at th_c.
step_size = th_base(2)-th_base(1);
idx_before = floor((th_c-th_base(1))/step_size) + 1; % 1-indexing
G_c = (Gamma(idx_before+1)-Gamma(idx_before))/step_size * ...
    (th_c - th_base(idx_before)) + Gamma(idx_before);
P_c = (Psi(idx_before+1)-Psi(idx_before))/step_size * ...
    (th_c - th_base(idx_before)) + Psi(idx_before);

constrData = struct;
constrData.step_l = s_l;
constrData.step_h = s_h;
constrData.Gamma_c = G_c;
constrData.Psi_c = P_c;
constrData.theta_c = th_c;
constrData.Gamma_f = Gamma(end);
constrData.Psi_f = Psi(end);
constrData.alpha_p = alpha_p;
constrData.theta_p = theta_p;

end