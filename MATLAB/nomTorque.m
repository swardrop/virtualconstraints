function u = nomTorque(cd, d_th_sq_0)
%NOMTORQUE Calculates the nominal torque required to maintain zero
% dynamics of the compass-gait walker given initial theta-dot^2
%   Output is a grid of torques corresponding to grid within cd.
%   This should ideally be replaced by some kind of polynomial.

d_theta_sq = cd.Gamma*d_th_sq_0 + cd.Psi;
dd_theta = -cd.beta./cd.alpha.*d_theta_sq - cd.gamma./cd.alpha;

qdd = cd.d_Phi.*[dd_theta; dd_theta] + cd.dd_Phi.*[d_theta_sq; d_theta_sq];
u = zeros(size(actuated(cd.Phi)));

for i = 1 : size(u, 2)
    [M, C, G, B] = dynMatrices(cd.Phi(:,i), cd.d_Phi(:,i));

    Bu = M*qdd(:,i) + C*cd.d_Phi(:,i)*d_theta_sq(i) + G;
    u(:,i) = pinv(B)*Bu;         % Pseudoinverse: I think this should work?
                                 % It works for the CG at least.
end

end