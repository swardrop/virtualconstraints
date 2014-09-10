function qp = delq(q)
% Relabeling of coordinates upon impact
% NOTE - This diverges from Westervelt's definition due to the constant
% terms, and as such is presented as a function of q rather than a constant
% matrix which multiplies q.

t1 = q(1);
t2 = q(2);
qp(1,1) = mod(t1 + t2 - pi, pi);
qp(2,1) = -2*pi - t2;
end

