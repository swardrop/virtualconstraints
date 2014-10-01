function constrData = makeConstr(theta_p, alpha_p)
% Takes a set of Bezier control points which are specified in each row of
% the points argument. Returns a structure containing the following fields:
%
% alpha_p - Bezier coefficients (identical to supplied points)
% theta_p - The start and end points of the phase variable (along with
%           inferred intermediate values
% step_l - length of step
% step_h - height of step
% theta_c - critical phase angle
% Gamma_c - value of Gamma function at critical theta
% Psi_c - value of Psi function at critical theta
% Gamma_f - value of Gamma function just before impact
% Psi_f - value of Psi function just before impact
% Gamma_p - value of Gamma function just after impact
% Psi_p - value of Psi function just after impact
%
% ADDITIONAL ITEMS WHICH SHOULD BE REMOVED FOR SPACE EFFICIENCY
% Gamma
% Psi
% th_base
% alpha
% beta
% gamma
% Phi
% d_Phi
% dd_Phi

% Calculate partial solution
[Gamma, Psi, th_base, th_c, alpha, beta, gamma, Phi, d_Phi, dd_Phi] = ...
    PartialSolZeroDyn(theta_p, alpha_p);

% Calculate step length and height
p2 = endSwingFoot(Phi(:,end), [0, 0]);

% Calculate Gamma and Psi at th_c.
step_size = th_base(2)-th_base(1);
idx_before = floor((th_c-th_base(1))/step_size) + 1; % 1-indexing
G_c = (Gamma(idx_before+1)-Gamma(idx_before))/step_size * ...
    (th_c - th_base(idx_before)) + Gamma(idx_before);
P_c = (Psi(idx_before+1)-Psi(idx_before))/step_size * ...
    (th_c - th_base(idx_before)) + Psi(idx_before);

% Calculate Gamma and Psi at th_p.
delqd = impactMatrices(Phi(:,end));
[~,~,c] = constrMatrices;
delthd = (c*delqd*d_Phi(:,end))^2;
G_p = delthd*Gamma(end);
P_p = delthd*Psi(end);

constrData = struct;
constrData.theta_p = theta_p;
constrData.alpha_p = alpha_p;
constrData.step_l = p2(1);
constrData.step_h = p2(2);
constrData.Gamma_c = G_c;
constrData.Psi_c = P_c;
constrData.theta_c = th_c;
constrData.Gamma_f = Gamma(end);
constrData.Psi_f = Psi(end);
constrData.Gamma_p = G_p;
constrData.Psi_p = P_p;

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