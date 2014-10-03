function [b0, b_th0, b1] = invarianceCond(th_a, al_a, b_thf, num_b, self)
%invarianceCond Produces the first two control pts (b0 and b1) of a Bezier
% constraint, along with the corresponding theta value for b0, given:
%   - the previous constraint (only the final two points, the start theta
%     and end theta values are used)
%   - the final theta value and the number of points in the constraint
%   - whether or not the prior constraint is the current constraint, in
%     which case the calculation is altered slightly.

if nargin == 3
    self = false;
end

[H, H0, c] = constrMatrices;

% First condition - the initial configuration of the constraint must equal
% the post-impact configuration of the preceding constraint.
q_postimpact = H*(delq/H)*[al_a(:,end); th_a(end)];
b_th0 = q_postimpact(end);
b0 = q_postimpact(1:end-1);
if (self)
    th_a(1) = b_th0;
end
% Second condition - the initial velocity of the constraint must equal the
% post-impact velocity of the preceding constraint.
[~, Phi_f, ~, dPhi_f] = constrEndPts(th_a, al_a);
delqd = impactMatrices(Phi_f);
b1 = H0*delqd*dPhi_f*((b_thf-b_th0)/(num_b-1))/(c*delqd*dPhi_f) + b0;
end