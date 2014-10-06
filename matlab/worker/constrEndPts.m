function [Phi_0, Phi_f, dPhi_0, dPhi_f] = constrEndPts(theta_p, alpha_p)
%constrEndPts The configuration and rate of change wrt the phase variable
% of the constraint given in {theta_p, alpha_p} at the endpoints.
H = constrMatrices;
Phi_0 = H\[alpha_p(:,1);theta_p(1)];
Phi_f = H\[alpha_p(:,end);theta_p(end)];
dPhi_0 = H\[(size(alpha_p,2)-1)/(theta_p(end)-theta_p(1)) ...
            *(alpha_p(:,2)-alpha_p(:,1)); 1];
dPhi_f = H\[(size(alpha_p,2)-1)/(theta_p(end)-theta_p(1)) ...
            *(alpha_p(:,end)-alpha_p(:,end-1)); 1];
end