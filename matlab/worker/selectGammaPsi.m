function [G_c, P_c, G_p, P_p] = ...
    selectGammaPsi(theta_p, alpha_p, Gamma, Psi, th_c, th_base)

% Calculate Gamma and Psi at th_c.
if th_c == inf
    if Psi(end) < 0
        G_c = Gamma(end);
        P_c = Psi(end);
    else
        G_c = Gamma(1);
        P_c = Psi(1);
    end
else
    step_size = th_base(2)-th_base(1);
    idx_before = floor((th_c-th_base(1))/step_size) + 1; % 1-indexing
    G_c = (Gamma(idx_before+1)-Gamma(idx_before))/step_size * ...
        (th_c - th_base(idx_before)) + Gamma(idx_before);
    P_c = (Psi(idx_before+1)-Psi(idx_before))/step_size * ...
        (th_c - th_base(idx_before)) + Psi(idx_before);
end

% Calculate Gamma and Psi at th_p.
[~, Phi_f, ~, dPhi_f] = constrEndPts(theta_p, alpha_p);
delqd = impactMatrices(Phi_f);
[~,~,c] = constrMatrices;
delthd = (c*delqd*dPhi_f)^2;
G_p = delthd*Gamma(end);
P_p = delthd*Psi(end);

end