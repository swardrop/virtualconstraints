function [b0, b_th0, b1] = invarianceCond( constr_a, b_thf, num_b, self )
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

% First condition - the initial configuration of the constraint must equal
% the post-impact configuration of the preceding constraint.
q_postimpact = delq([constr_a(end,1); constr_a(end,2)]);
b_th0 = q_postimpact(1);
b0 = q_postimpact(2);
if (self)
    constr_a(1,2) = b0;
end
% Second condition - the initial velocity of the constraint must equal the
% post-impact velocity of the preceding constraint.
c = [1 0];
H0 = [0 1];
H = [c; H0];
omega_a = H\[1; size(constr_a,1)/(constr_a(end,1)-constr_a(1,1)) ...
            * (constr_a(end,2) - constr_a(end-1,2))];
delqd = impactMatrices([constr_a(end,1); constr_a(end,2)]);
b1 = H0*delqd*omega_a*((b_thf-b_th0)/num_b)/(c*delqd*omega_a) + b0;
end

