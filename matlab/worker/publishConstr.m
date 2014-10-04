function constr = publishConstr(theta_p, alpha_p, initq, finalq, num_points)
% Creates a constraint storing only the data required in the VC libary
% alpha_p - Bezier coefficients (identical to supplied points)
% theta_p - The start and end points of the phase variable (along with
%           inferred intermediate values
% Gamma_c - value of Gamma function at critical theta
% Psi_c - value of Psi function at critical theta
% Gamma_f - value of Gamma function just before impact
% Psi_f - value of Psi function just before impact
% Gamma_p - value of Gamma function just after impact
% Psi_p - value of Psi function just after impact
% initq - index into impact configuration array of initial cond
% finalq - index into impact configuration array of final cond

if nargin < 3
    initq = 0;
    finalq = 0;
    if nargin < 5
        num_points = 2000;
    end
end

% Calculate partial solution
[Gamma,Psi,th_base,th_c] = PartialSolZeroDyn(theta_p, alpha_p, num_points);

[G_c, P_c, G_p, P_p] = ...
    selectGammaPsi(theta_p, alpha_p, Gamma, Psi, th_c, th_base);

constr = struct;
constr.theta_p = theta_p;
constr.alpha_p = alpha_p;

constr.Gamma_c = G_c;
constr.Psi_c = P_c;
constr.Gamma_f = Gamma(end);
constr.Psi_f = Psi(end);
constr.Gamma_p = G_p;
constr.Psi_p = P_p;

constr.initq = initq;
constr.finalq = finalq;

end