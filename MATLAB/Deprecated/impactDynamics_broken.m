function [t1_, t2_, t1d_, t2d_] = impactDynamics(t1, t2, t1d, t2d)

[Is, ls, ms] = getDynParams();
I1 = Is(1);
I2 = Is(2);
l1 = ls(1);
l2 = ls(2);
m1 = ms(1);
m2 = ms(2);

% Position - simple impact map
t1_ = mod(t1 + t2 - pi, pi);
t2_ = -2*pi - t2;

% Velocity:
M_s = [I1+I2+m2*(l1^2+0.25*l2^2+l1*l2*cos(t2)), ...
        I2+0.5*m2*(0.5*l2^2+l1*l2*cos(t2)); ...
        I2+0.5*m2*(0.5*l2^2+l1*l2*cos(t2)), ...
        I2+0.25*m2*l2^2];
    
M_e_comp = [m2*(l1*sin(t1)+l2/2*sin(t1+t2)), ...
    m2*(l1*cos(t1) + l2/2*cos(t1+t2)); ...
    -0.5*m2*l2*sin(t1+t2) 0.5*m2*l2*cos(t1+t2)];

M_e = [M_s, M_e_comp; M_e_comp', (m1+2*m2)*eye(2)];

E2 = [-l1*sin(t1)-l2*sin(t1+t2), -l2*sin(t1+t2), 1, 0;
    l1*cos(t1)+l2*cos(t1+t2), l2*cos(t1+t2), 0, 1];

delF2 = -(E2*(M_e\E2'))\E2*[eye(2);zeros(2)];
delqed = M_e\E2'*delF2 + [eye(2);zeros(2)];

result = [delqed; delF2]*[t1d; t2d];

t1d_ = result(1);
t2d_ = result(2);
t1d_ = t1d_ + t2d_;
t2d_ = -t2d_;

% Perhaps check for validity? Forces must be in static friction cone
F2_T = result(3);
F2_N = result(4);


fprintf('t1+ = %.2f, t2+ = %.2f, t1d+ = %.2f, t2d+ = %.2f\n\n', ...
    t1_, t2_, t1d_, t2d_);
end