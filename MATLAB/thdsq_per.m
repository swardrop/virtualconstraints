function thdsq = thdsq_per(cd)
% Returns the velocity which, when set as the initial velocity and assuming
% perfect constraint regulation, is equal to the post-impact velocity of
% the constraint.

Phi_thf = bezConstraint(cd.theta_p, cd.alpha_p, cd.theta_p(end));
delthd = impactMatrices(Phi_thf);
[~,~,c] = constrMatrices;

thdsq = cd.Psi_f/((c*delthd*Phi_thf)^(-2) - cd.Gamma_f);
end