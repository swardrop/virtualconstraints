function [theta_p, alpha_p] = ...
    optimiseConstraint(start, fin, DelKE, deg)
% optimiseConstraint Produces the optimal constraint given the target final
% kinetic energy subject to the start and end conditions for the
% compass-gait walker
%
% Optimality is considered to be the minimum of integrated square of the
% input torque.
% Note start and end are column vectors with two elements
% constrPts will be a 2xdeg matrix with the first and last column equal to
% the start and end conditions.

global desired_DelKE
desired_DelKE = DelKE;

x0 = getInitEstimate(start, fin, deg);

[Aeq, beq] = eqConstraints(start, fin, deg);
[lb, ub] = bounds(start, fin, deg);

x = fmincon(@cost, x0, [], [], Aeq, beq, lb, ub, @nonlconstrs);

alpha_p = x(1:end-2);
alpha_p = reshape(alpha_p, length(alpha_p)/deg, deg);
theta_p(1) = x(end-1);
theta_p(2) = x(end);
theta_p = linspace(theta_p(1), theta_p(2), deg);
end

% PointData is [alpha(:); theta+; theta-]
function J = cost(pointData)
% Generate theta_p and alpha_p from pointData

% Calculate nominal intial squared velocity

% Get integral of squared input torque
J = squareIntTorque(theta_p, alpha_p, thd2);

end

% Get inital estimate of coefficients based upon start and endpoints
function x0 = getInitEstimate(start, fin, deg)

end

% Equality constraints - start and end conditions + zero slope at impact
function [Aeq, beq] = eqConstraints(start, fin, deg)

end

% Bounds on alpha vals
function [lb, ub] = bounds(start, fin, deg)

end

% Nonlinear inequality and equality constraints
function [c, ceq] = nonlconstrs(pointData)
global desired_DelKE
% Note c(x) <= 0 and ceq(x) = 0 are tested in fmincon
% c (ineq)  - Conditions 3 - 5.
% ceq       - Invariance and KE addition/subtraction
end

function Su2 = squareIntTorque(theta_p, alpha_p, thd2)

gridN = 20;
theta = linspace(theta_p(1), theta_p(end), gridN);
% WHATEVER....

Su2 = 0;

end