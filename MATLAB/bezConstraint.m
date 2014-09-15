function q = bezConstraint(theta_p, alpha_p, theta)
% Produces the nominal value of the state evaluated at theta given the
% constraint defined by theta_p and alpha_p

s = (theta - theta_p(1)) / (theta_p(end) - theta_p(1));
s(s>1) = 1;
s(s<0) = 0;
q_dep = zeros([size(alpha_p, 1), 1]);
n = size(points, 2) - 1;
for i = 0 : n
    q_dep = q_dep + nchoosek(n,i) * (1-s)^(n-i) * s^i * alpha_p(:,i+1);
end


% FOR COMPASS-GAIT - phase variable is q2.
q = [q_dep; theta];