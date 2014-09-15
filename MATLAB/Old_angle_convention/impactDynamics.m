function [q_, qd_, status] = ...
    impactDynamics(q, qd)

% Position - simple impact map
q_ = delq(q);

% Velocity:
[delqd, delF2] = impactMatrices(q);
qd_ = delqd*qd;
F2 = delF2*qd;


% Print diagnostic information
fprintf(['t1- = %.2f; \t   t1+ = %.2f\n' ...
         't2- = %.2f; \t   t2+ = %.2f\n' ...
         't1d- = %.2f;\t   t1d+ = %.2f\n' ...
         't2d- = %.2f;\t   t2d+ = %.2f\n' ...
         'F2_T = %.2f;\t   F2_N = %.2f\n\n'], ...
        q(1), q_(1), q(2), q_(2), ...
        qd(1), qd_(1), qd(2), qd_(2), ...
        F2(1), F2(2));
    
status = 0;
if abs(F2(1)) > 0.7*F2(2)
    disp('Warning: Likely slippage! Requires coefficient > 0.7');
    status = 1;
end
if F2(2) < 0
    disp('Negative reaction force at impact, something has gone wrong');
    status = 1;
end
end