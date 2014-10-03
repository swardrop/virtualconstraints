function thdsq = thdsq_nom(cd, DelKE)
% Returns nominal initial velocity for the given constraint for use in
% determining the nominal torque required to maintain the constraint.

ratio = 1.2;

[~, Phi_f, ~, dPhi_f] = constrEndPts(cd.theta_p, cd.alpha_p);
delqd = impactMatrices(Phi_f);
[~,~,c] = constrMatrices;
delthd = c*delqd*dPhi_f;

if DelKE < 0
    % Fixed factor over minimum feasible velocity after impact
    thdsq = (delthd^(-2)*(-ratio^2*cd.Psi_c/cd.Gamma_c) - cd.Psi_f)/cd.Gamma_f;
else %if DelKE >= 0
    thdsq = -ratio^2*cd.Psi_c/cd.Gamma_c;
end