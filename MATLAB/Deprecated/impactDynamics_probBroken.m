function [t1_, t2_, t1d_, t2d_] = impactDynamics_probBroken(t1, t2, t1d, t2d)

[Is, ls, ms] = getDynParams();
I = Is(2);      % Moment of inertia about centre
l = ls(1);
m = ms(1);

% Position - simple impact map
t1_ = mod(t1 + t2 - pi, pi);
t2_ = -2*pi - t2;

% Velocity:
M_e = [2*I+m*l^2*(1.5+cos(t2)),         I+0.5*m*l^2*(0.5+cos(t2)), ...
      -m*l/2*(3*sin(t1)+sin(t1+t2)),    m*l/2*(3*cos(t1)+cos(t1+t2)); ...
      I+0.5*m*l^2*(0.5+cos(t2)),        I + m*l^2/4, ...
      -0.5*m*l*sin(t1+t2),              0.5*m*l*cos(t1+t2); ...
      -m*l/2*(3*sin(t1)+sin(t1+t2)),    -0.5*m*l*sin(t1+t2), ...
      2*m,                              0; ...
      m*l/2*(3*cos(t1)+cos(t1+t2)),     0.5*m*l*cos(t1+t2), ...
      0,                                2*m];

E2 = [-l*sin(t1)-l*sin(t1+t2), -l*sin(t1+t2), 1, 0;
    l*cos(t1)+l*cos(t1+t2), l*cos(t1+t2), 0, 1];

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