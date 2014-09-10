function [delqd, delF2] = impactMatrices(q)

t1 = q(1);
t2 = q(2);

[Is, ls, ms] = getDynParams();
I = Is(2);      % Moment of inertia about centre
l = ls(1);
m = ms(1);

A = m*l^2/8*[2*(cos(t2)+1), cos(t2)+1; cos(t2)+1, 1] + I*[2 1; 1 1];
M_e = [A, zeros(2); zeros(2), 2*m*eye(2)];

E2 = [-l*sin(t1)-l*sin(t1+t2), -l*sin(t1+t2), 1, 0;
    l*cos(t1)+l*cos(t1+t2), l*cos(t1+t2), 0, 1];

delF2 = -(E2*(M_e\E2'))\E2*[eye(2);zeros(2)];
delqed = M_e\E2'*delF2 + [eye(2);zeros(2)];
R = [1 1; 0 -1];
delqd = [R zeros(2)]*delqed;

end