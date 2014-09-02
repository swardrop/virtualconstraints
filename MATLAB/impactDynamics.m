function [t1_, t2_, t1d_, t2d_] = impactDynamics(t1, t2, t1d, t2d)

[Is, ls, ms] = getDynParams();
I = Is(2);      % Moment of inertia about centre
l = ls(1);
m = ms(1);

% Position - simple impact map
t1_ = mod(t1 + t2 - pi, pi);
t2_ = -2*pi - t2;

% Velocity:
A = m*l^2/8*[2*(cos(t2)+1), cos(t2)+1; cos(t2)+1, 1] + I*[2 1; 1 1];
M_e = [A, zeros(2); zeros(2), 2*m*eye(2)];

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


fprintf(['t1- = %.2f; \t   t1+ = %.2f\n' ...
         't2- = %.2f; \t   t2+ = %.2f\n' ...
         't1d- = %.2f;\t   t1d+ = %.2f\n' ...
         't2d- = %.2f;\t   t2d+ = %.2f\n' ...
         'F2_T = %.2f;\t   F2_N = %.2f\n\n'], ...
        t1, t1_, t2, t2_, t1d, t1d_, t2d, t2d_, F2_T, F2_N);
    
if F2_T > 0.7*F2_N
    disp('Warning: Likely slippage! Requires coefficient > 0.7');
end
end