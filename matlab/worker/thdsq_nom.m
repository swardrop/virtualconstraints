function thdsq = thdsq_nom(cd)
% Returns nominal initial velocity for the given constraint for use in
% determining the nominal torque required to maintain the constraint.

ratio = 1.2;

[~, Phi_f, ~, dPhi_f] = constrEndPts(cd.theta_p, cd.alpha_p);
Delqd = impactMatrices(Phi_f);
[~,~,c] = constrMatrices;
Delthd = c*Delqd*dPhi_f;

[Gamma_c, Psi_c] = selectGammaPsi(cd.theta_p, cd.alpha_p, ...
    cd.Gamma, cd.Psi, cd.th_c, cd.th_base);

thdsq = max((Delthd^(-2)*(-ratio^2*Psi_c/Gamma_c) - ...
    cd.Psi(end))/cd.Gamma(end), ...
    -ratio^2*Psi_c/Gamma_c);
end