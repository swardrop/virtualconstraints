function inv_chk = invCndChk(th_a, al_a, th_b, al_b )
%invCndChk Returns true if the second constraint meets the criteria for
% invariance w.r.t the first constraint, otherwise returns false.

c = [0 1];      % For compass-gait model
H0 = [1 0];     % For compass-gait model
H = [H0; c];

% First condition - the initial configuration of the constraint must equal
% the post-impact configuration of the preceding constraint.
q_postimpact = H*(delq()/H)*[al_a(:,end); th_a(end)];
b_th0 = q_postimpact(end);
b0 = q_postimpact(1:end-1);
if th_b(1) ~= b_th0 || al_b(:,1) ~= b0
    inv_chk = false;
    return;
end

% Second condition - the initial velocity of the constraint must equal the
% post-impact velocity of the preceding constraint.
omega_a = H\[length(th_a)/(th_a(end)-th_a(1)) ...
            * (al_a(:,end) - al_a(:,end-1)); 1];
delqd = impactMatrices(bezConstraint(th_a, al_a, th_a(end)));
b1 = H0*delqd*omega_a*...
    ((th_b(end)-th_b(1))/length(th_b))/(c*delqd*omega_a) + b0;
if al_b(:,2) ~= b1
    inv_chk = false;
else
    inv_chk = true;
end
end

