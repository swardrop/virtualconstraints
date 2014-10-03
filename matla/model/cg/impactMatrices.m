function [delqd, delF2] = impactMatrices(q)

% Impact dynamics for compass-gait.

[I, l, m] = dynParams();

A = m*l^2/8*[1, cos(q(1))-1; cos(q(1))-1, 2*(1-cos(q(1)))] + I*[1 -1; -1 2];
M_e = [A, zeros(2); zeros(2), 2*m*eye(2)];

E2 = [0.75*l*cos(q(1)-q(2)), 0.25*l*(cos(q(2)) - 3*cos(q(1)-q(2))), 1, 0;
     0.75*l*sin(q(1)-q(2)), -0.25*l*(sin(q(2)) + 3*sin(q(1)-q(2))), 0, 1];

dYe = 0.25*l*[cos(q(1)-q(2)), 3*cos(q(2))-cos(q(1)-q(2));
              sin(q(1)-q(2)), -3*sin(q(2))-sin(q(1)-q(2))];

delF2 = -(E2*(M_e\E2'))\E2*[eye(2);dYe];
delqed = M_e\E2'*delF2 + [eye(2);dYe];
R = delq();
delqd = [R zeros(2)]*delqed;


% % From Westervelt pg 65-66; lc = l/2
% q1 = q(1);
% den = I^2 + (5*l^4*m^2)/16 - (cos(q1)^2*l^4*m^2)/4 + (3*I*l^2*m)/2;
% Dqd11 = ((l^2*m)/4 - I)*(I + (l^2*m)/4 - (cos(q1)*l^2*m)/2);
% Dqd12 = -(l^2*m*(2*I - 2*I*cos(q1) - (l^2*m*cos(q1))/2 + ...
%     (l^2*m*cos(2*q1))/2))/2;
% Dqd21 = ((l^2*m)/4 - I)*((m*l^2)/4 + I);
% Dqd22 = I^2 - (l^4*m^2)/16 + (l^4*m^2*cos(q1))/8 + ...
%     (3*I*l^2*m*cos(q1))/2;
% Dqd = 1/den*[Dqd11 Dqd12; Dqd21 Dqd22];
% I have confirmed that this matches delqd!
end