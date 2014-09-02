function y = solveBez(th_1, t, points)

P0 = points(:,1);
P1 = points(:,2);
P2 = points(:,3);
P3 = points(:,4);

%t = solveBezFor_t(points);
subs(t);
t = eval(t);
t = t(imag(t) == 0);
y = P0(2)*((1 - t).^3) + 3*P1(2)*(((1 - t).^2).*t) + ...
    3*P2(2)*((1 - t).*t.^2) + P3(2)*(t.^3);
