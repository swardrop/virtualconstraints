function th2_des = holonomic(th_1, points)
tic
th2_des = NaN;
P0 = points(:,1);
P1 = points(:,2);
P2 = points(:,3);
P3 = points(:,4);
global t_bez
% % Solve Bezier equations to find th_2 for given th_1
% % (Assume th_2 may be expressed as function of th_1 - this code must be
% % substantially changed to allow for multiple th_2 values for one th_1
% % value) <- this assumption is fairly safe for walking motion - if th_1
% % doubles back on itself, it is likely that the motion has failed.
% syms r;
% % Find explicit expression for t
% t = solve(th_1 == P0(1)*((1 - r).^3) + 3*P1(1)*(((1 - r).^2).*r) + ...
%     3*P2(1)*((1 - r).*r.^2) + P3(1)*(r.^3), r);
% % Choose real t
% t = t(imag(t) == 0);
if (th_1 > 20)
    disp(th_1);
end
t = subs(t_bez);
t = t(imag(t) < 0.000001);
t = real(eval(t));
t = t(t<=1);
t = t(t>=0);

if isempty(t)
    return
end

% Substitute expression for t into equation for th2.
th2_des = P0(2)*((1 - t).^3) + 3*P1(2)*(((1 - t).^2).*t) + ...
    3*P2(2)*((1 - t).*t.^2) + P3(2)*(t.^3);
toc
end