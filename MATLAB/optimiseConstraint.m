function constraintData = ...
    optimiseConstraint(start, fin, DelKE, sigma, deg)
% optimiseConstraint Produces the optimal constraint given the target
% change in post-impact kinetic energy from the previous post-impact KE
% subject to the start and end conditions.
%
% Optimality is considered to be the minimum of integrated square of the
% input torque.
% Note start and fin are column vectors [q1; q2; ... ; qn]

global desired_DelKE sigmax degree theta_ends alpha_ends
desired_DelKE = DelKE;
degree = deg;
sigmax = sigma;

theta_ends = [phasevar(start), phasevar(fin)];
alpha_ends(:,1) = actuated(start);
alpha_ends(:,4) = actuated(fin);
alpha_ends(:,3) = alpha_ends(:,end); % Zero slope at impact
[~, ~, al1] = invarianceCond(theta_ends, alpha_ends, theta_ends(end), ...
                degree+1, false);
alpha_ends(:,2) = al1;

x0 = getInitEstimate(start, fin, deg);
[lb, ub] = bounds(start, fin, deg);

options = optimoptions('fmincon', 'Algorithm', 'interior-point');
x = fmincon(@cost, x0, [], [], [], [], lb, ub, @nonlconstrs, options);

[theta_p, alpha_p] = getCoefficients(x);
constraintData = makeConstr(theta_p, alpha_p);

end

function J = cost(x)
global desired_DelKE
% Extract the Bezier coefficients from x
[theta_p, alpha_p] = getCoefficients(x);
% Calculate nominal intial squared velocity
cd = getOrMakeConstr(theta_p, alpha_p);
thd2 = thdsq_nom(cd, desired_DelKE);
% Get integral of squared input torque
J = squareIntTorque(cd, thd2);
end

% Get inital estimate of coefficients based upon start and end points
% This just has the points lying on the straight line between the two
% end points in the Bezier coordinate space.
function x0 = getInitEstimate(start, fin, deg)
bezStart = actuated(start);
bezFin = actuated(fin);
for i = length(bezStart):-1:1
    alpha_p(i,:) = linspace(bezStart(i,1), bezFin(i,end), deg+1);
end
x0 = alpha_p(:,3:end-2);
x0 = x0(:);
end

% Bounds on alpha vals
function [lb, ub] = bounds(start, fin, deg)
lb = [];
ub = [];
end

% Nonlinear inequality and equality constraints
% ceq -  KE addition/subtraction
% Note c(x) <= 0 and ceq(x) = 0 are tested in fmincon
function [c, ceq] = nonlconstrs(x)
global sigmax desired_DelKE
[theta_p, alpha_p] = getCoefficients(x);
[~, Phi_f, dPhi_0, dPhi_f] = constrEndPts(theta_p, alpha_p);

cd = getOrMakeConstr(theta_p, alpha_p);
thdsq_0 = thdsq_nom(cd, desired_DelKE);
td2m = cd.Gamma_f*thdsq_0 + cd.Psi_f;
Delqd = impactMatrices(Phi_f);
M = dynMatrices(delq*Phi_f, dPhi_f);

KE_after = (Delqd*dPhi_f)' * M * Delqd*dPhi_f * td2m;
KE_before = dPhi_0' * M * dPhi_f * thdsq_0;
DelKE = KE_after - KE_before;

ceq = desired_DelKE - DelKE;

if size(alpha_p,1) < 2
    % Don't impose ground constraint for CG
    c = [];
else
    for i = length(theta_p)-1 : -1 : 2
        p = endSwingFoot(bezConstraint(theta_p, alpha_p, theta_p(i)), ...
            [0,0]);
        ind = find(p(1) > sigmax(:,1), 1);
        height = sigmax(ind, 2);
        c(i-1) = height - p(2);
    end
end
end

function Su2 = squareIntTorque(cd, thd2)
u = nomTorque(cd, thd2)';
Su2 = trapz(sqrt(sum(u.^2)));
end

% x is (alpha(:,3:end-2))(:)
function [theta_p, alpha_p] = getCoefficients(x)
global degree alpha_ends theta_ends
theta_p = linspace(theta_ends(1), theta_ends(2), degree+1);

alpha_p_infix = reshape(x, length(x)/(degree-3), degree-3);
alpha_p = [alpha_ends(:,1:2), alpha_p_infix, alpha_ends(:,3:4)];
end

function cd = getOrMakeConstr(theta_p, alpha_p)
persistent constr
if ~isempty(constr) 
    if constr.alpha_p == alpha_p
        cd = constr;
        return;
    end
end
cd = makeConstr(theta_p, alpha_p);
constr = cd;
end
        