function thdsq = thdsq_nom(cd, DelKE)
% Returns nominal initial velocity for the given constraint for use in
% determining the nominal torque required to maintain the constraint.

ratio = 1.2;

[~, Phi_f, ~, dPhi_f] = constrEndPts(cd.theta_p, cd.alpha_p);
delqd = impactMatrices(Phi_f);
[~,~,c] = constrMatrices;
delthd = c*delqd*dPhi_f;

[Gamma_c, Psi_c] = selectGammaPsi(cd.theta_p, cd.alpha_p, ...
    cd.Gamma, cd.Psi, cd.th_c, cd.th_base);

if DelKE <= 0
    % Fixed factor over minimum feasible velocity after impact
    thdsq = (delthd^(-2)*(-ratio^2*Psi_c/Gamma_c) - cd.Psi(end))/cd.Gamma(end);
else %if DelKE > 0
    thdsq = -ratio^2*Psi_c/Gamma_c;
end