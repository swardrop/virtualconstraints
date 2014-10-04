function idxs = sortConstrs(constrs, targetvel)
% At the moment, just sort based upon target critical velocity.
% Later, perhaps attempt multiple orderings such that all velocities are
% covered, or perhaps even a total ordering.

req_init_vels = (targetvel^2 - [constrs.Psi_c])./[constrs.Gamma_c];
[~,idxs] = sort(req_init_vels);

end