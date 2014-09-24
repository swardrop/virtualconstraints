function thdsq = thdsq_nom(cd)
% Returns nominal initial velocity for the given constraint for use in
% determining the nominal torque required to maintain the constraint.

Phi_thf = cd.Phi(:,end);
d_Phi_thf = cd.d_Phi(:,end);
delqd = impactMatrices(Phi_thf);
[~,~,c] = constrMatrices;
delthd = c*delqd*d_Phi_thf;

% Periodic velocity
% thdsq = cd.Psi_f/((delthd)^(-2) - cd.Gamma_f)

% Fixed velocity after impact
% thdsq_p = 2.5;
% thdsq = ( (thdsq_p/delthd^2) - cd.Psi_f ) / cd.Gamma_f;

% Fixed factor over minimum feasible velocity after impact
ratio = 1.2;
thdsq = (delthd^(-2)*(-ratio^2*cd.Psi_c/cd.Gamma_c) - cd.Psi_f)/cd.Gamma_f;

end