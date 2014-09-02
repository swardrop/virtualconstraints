function r = solveBezFor_t(points)

P0 = points(:,1);
P1 = points(:,2);
P2 = points(:,3);
P3 = points(:,4);

% Solve Bezier equations to find th_2 for given th_1
% (Assume th_2 may be expressed as function of th_1 - this code must be
% substantially changed to allow for multiple th_2 values for one th_1
% value) <- this assumption is fairly safe for walking motion - if th_1
% doubles back on itself, it is likely that the motion has failed.
syms r th_1;
% Find explicit expression for t
r = solve(th_1 == P0(1)*((1 - r).^3) + 3*P1(1)*(((1 - r).^2).*r) + ...
    3*P2(1)*((1 - r).*r.^2) + P3(1)*(r.^3), r);

r = simple(r);

end