function [theta_p, alpha_p] = optimiseConstraintCG(targetKE, start, fin)
% optimiseConstraint Produces the optimal constraint given the target final
% kinetic energy subject to the start and end conditions for the
% compass-gait walker
%
% Optimality is considered to be the minimum of integrated input torque.
% Note start and end are column vectors with two elements
% constrPts will be a 2x6 matrix with the first and last column equal to
% the start and end conditions.
%
% Note that element 1 of the start and end coordinates must be the phase
% variable.

gridN = 100;
grid_phase = linspace(start(1), fin(1), gridN);

% Maybe we can take some kind of reference progression of theta1? 
% Need to investigate how much theta2 progression affects t1 and if
% such a reference is even possible -- also even less sure about general
% case, though it's true that for walking gaits, we're always going to have
% a monotonic theta1 at least.


cvx_begin
    variables p(1,4)
    % Minimise integrated torque over the gridded constraint
    %minimise(sum(----))
    subject to
        % KE at end == targetKE
        % Maybe have some limit on the values of the control points for
        % feasibility
cvx_end

% Or maybe use Newton's method with some barrier functions?

end

