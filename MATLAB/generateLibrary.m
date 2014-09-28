function lib = generateLibrary
%generateLibrary Produces a complete libary of motion primitives for the
% robot whose dynamics are expressed in the functions dynMatrices and
% impactMatries and which have control applied as defined in
% constrMatrices, where H0*q are the actuated coordinates and c*q is the
% unactuated coordinate.
%   lib is a structure? with the following fields:
%   - constraints - a cell array containing all constraints, indexed by
%   step length and height
%   - orderings - a cell array containing information regarding the
%   ordering between each constraint?


end

% Nonlinear inequality and equality constraints
% c (ineq)  - Conditions 3 - 5.
% ceq       - Invariance (Cond 1&2)
% Note c(x) <= 0 and ceq(x) = 0 must be true or else the constraint is
% invalid.
function [c, ceq] = nonlconstrs(x)
global desired_DelKE theta_ends
[theta_p, alpha_p] = getCoefficients(x);
[Phi_0, Phi_f, dPhi_0, dPhi_f] = constrEndPts(theta_p, alpha_p);

% Note condition 1 must be enforced on-line.

% Condition 2

% Condition 3a
[~, dp2] = endSwingFoot(Phi_0, [0,0]);
c(4) = -[0,1]*dp2*dPhi_0;

% Condition 3b
delqd = impactMatrices(Phi_f);
[~, dp2] = endSwingFoot(delq()*delqd);
c(3) = -[0,1]*dp2*delqd*d_Phi_f;

% Condition 4
c(2) = 0; %TODO

% Condition 5
c(1) = 0; %TODO
end