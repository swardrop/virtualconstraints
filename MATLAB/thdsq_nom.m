function thdsq = thdsq_nom(cd)
% Returns nominal initial velocity for the given constraint for use in
% determining the nominal torque required to maintain the constraint.

Phi_thf = cd.Phi(:,end);
d_Phi_thf = cd.d_Phi(:,end);
delqd = impactMatrices(Phi_thf);
[~,~,c] = constrMatrices;
delthd = c*delqd*d_Phi_thf;

% thdsq = cd.Psi_f/((delthd)^(-2) - cd.Gamma_f); Periodic

% Fixed velocity after impact
thdsq_p = 2.5;
thdsq = ( (thdsq_p/delthd^2) - cd.Psi_f ) / cd.Gamma_f;

end