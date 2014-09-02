function [Ts1, Ts2, same] = checkDynamics(t1,t1d,t1dd,t2,t2d,t2dd)

[I, l, m, g] = getDynParams();
s2 = sin(t2);
c2 = cos(t2);
c1 = cos(t1);
c12 = cos(t1 + t2);

% First expression: separated equations
T11 = t1dd*(I(1) + I(2) + m(2)*(l(1)^2 + 0.25*l(2)^2 + l(1)*l(2)*c2)) - ...
    t1d*t2d*(m(2)*l(1)*l(2)*s2) + t2dd*(I(2) + 0.5*m(2)*(0.5*l(2)^2 + l(1)*l(2)*c2)) ...
    - t2d^2*(0.5*m(2)*l(1)*l(2)*s2) + 0.5*l(1)*c1*m(1)*g + m(2)*g*(l(1)*c1 + 0.5*l(2)*c12);

T21 = t2dd*(I(2) + 0.25*m(2)*l(2)^2) + ...
    t1dd*(I(2) + 0.5*m(2)*(0.5*l(2)^2 + l(1)*l(2)*c2)) + t1d^2*(0.5*m(2)*l(1)*l(2)*s2) ...
    + 0.5*m(2)*g*l(2)*c12;
Ts1 = [T11;T21];
% Second expression: matrix form
M = [I(1)+I(2)+m(2)*(l(1)^2+0.25*l(2)^2+l(1)*l(2)*cos(t2)), ...
    I(2)+0.5*m(2)*(0.5*l(2)^2+l(1)*l(2)*cos(t2)); ...
    I(2)+0.5*m(2)*(0.5*l(2)^2+l(1)*l(2)*cos(t2)), ...
    I(2)+0.25*m(2)*l(2)^2];

C = sin(t2)*[-m(2)*l(1)*l(2)*t2d, -0.5*m(2)*l(1)*l(2)*t2d; ...
    0.5*m(2)*l(1)*l(2)*t1d, 0];

G = [0.5*l(1)*m(1)*g*cos(t1) + m(2)*g*(l(1)*cos(t1)+0.5*l(2)*cos(t1+t2))
    0.5*m(2)*g*l(2)*cos(t1+t2)];

Ts2 = M*[t1dd; t2dd] + C*[t1d;t2d] + G;

same = Ts1 == Ts2;