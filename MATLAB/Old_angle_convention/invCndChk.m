function inv_chk = invCndChk( constr_a, constr_b )
%invCndChk Returns true if the second constraint meets the criteria for
% invariance w.r.t the first constraint, otherwise returns false.

% First condition - the initial configuration of the constraint must equal
% the post-impact configuration of the preceding constraint.
q_postimpact = delq([constr_a(end,1); constr_a(end,2)]);
if constr_b(1,:)' ~= q_postimpact
    inv_chk = false;
    return;
end

% Second condition - the initial velocity of the constraint must equal the
% post-impact velocity of the preceding constraint.
c = [1 0];
H0 = [0 1];
H = [c; H0];
omega = H\[1; size(constr_a,1)/(constr_a(end,1)-constr_a(1,1)) ...
            * (constr_a(end,2) - constr_a(end-1,2))];
delqd = impactMatrices([constr_a(end,1); constr_a(end,2)]);
b1 = H0*delqd*omega*((constr_b(end,1)-constr_b(1,1))/size(constr_b,1)) /...
    (c*delqd*omega) + constr_b(1,2);
if constr_b(2,2) ~= b1
    inv_chk = false;
else
    inv_chk = true;
end
end

