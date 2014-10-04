function constrData = makeConstr(theta_p, alpha_p, num_points)
% Takes a set of Bezier control points which are specified in each row of
% the points argument. Returns a structure containing the following fields:
%
% alpha_p - Bezier coefficients (identical to supplied points)
% theta_p - The start and end points of the phase variable (along with
%           inferred intermediate values
% theta_c
% Gamma
% Psi
% th_base
% alpha
% beta
% gamma
% Phi
% d_Phi
% dd_Phi

if nargin < 3
    num_points = 25;
end

% Calculate partial solution
[Gamma, Psi, th_base, theta_c, alpha, beta, gamma, Phi, d_Phi, dd_Phi] = ...
    PartialSolZeroDyn(theta_p, alpha_p, num_points);

constrData = struct;
constrData.theta_p = theta_p;
constrData.alpha_p = alpha_p;
constrData.theta_c = theta_c;
constrData.Gamma = Gamma;
constrData.Psi = Psi;
constrData.th_base = th_base;
constrData.alpha = alpha;
constrData.beta = beta;
constrData.gamma = gamma;
constrData.Phi = Phi;
constrData.d_Phi = d_Phi;
constrData.dd_Phi = dd_Phi;

end