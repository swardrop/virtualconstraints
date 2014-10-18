function [constraintData, flag] = ...
    optimiseConstraint(start, fin, DelKE, sigma, deg, grid_num, reg)
% optimiseConstraint Produces the optimal constraint given the target
% change in post-impact kinetic energy from the previous post-impact KE
% subject to the start and end conditions.
%
% Optimality is considered to be the minimum of integrated square of the
% input torque.
% Note start and fin are column vectors [q1; q2; ... ; qn]

if nargin < 7
    reg = 1;
end

th_ends = [phasevar(start), phasevar(fin)];
al_ends(:,1) = actuated(start);
al_ends(:,4) = actuated(fin);
al_ends(:,3) = al_ends(:,end); % Zero slope at impact
[~, ~, al1] = invarianceCond(th_ends, al_ends, th_ends(end), ...
                deg+1, false);
al_ends(:,2) = al1;

x0 = getInitEstimate(start, fin, deg);
[lb, ub] = bounds(start, fin, deg);

options = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
    'Display', 'off');
[x,~,flag] = fmincon(...
    @(x)cost(x, th_ends, al_ends, DelKE, deg, grid_num, reg), ...
    x0, [], [], [], [], lb, ub, ...
    @(x)nonlconstrs(x, th_ends, al_ends, DelKE, sigma, deg, grid_num), ...
    options);

[theta_p, alpha_p] = getCoefficients(x, th_ends, al_ends, deg);
constraintData = publishConstr(theta_p, alpha_p);

end

function J = cost(x, th_ends, al_ends, DelKE, deg, grid_num, reg)
% Extract the Bezier coefficients from x
[theta_p, alpha_p] = getCoefficients(x, th_ends, al_ends, deg);
% Calculate nominal intial squared velocity
cd = getOrMakeConstr(theta_p, alpha_p, grid_num);
thd2 = thdsq_nom(cd);

% Get integral of squared input torque
u = nomTorque(cd, thd2);
%Su2 = trapz(sum(u, 1).^2); % Integral of 2-norm squared.
J = trapz(u.^2) + reg*norm(x);
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
lb = []; %-10*[0; ones(deg-4,1)];
ub = []; %10*ones(deg-3,1);
end

% Nonlinear inequality and equality constraints
% ceq -  KE addition/subtraction
% Note c(x) <= 0 and ceq(x) = 0 are tested in fmincon
function [c, ceq] = nonlconstrs(x, th_ends, al_ends, DelKE, ...
    sigma, deg, grid_num)
ceq = [];
[theta_p, alpha_p] = getCoefficients(x, th_ends, al_ends, deg);

cd = getOrMakeConstr(theta_p, alpha_p, grid_num);
thdsq_0 = thdsq_nom(cd);
td2m = cd.Gamma(end)*thdsq_0 + cd.Psi(end);
Delqd = impactMatrices(cd.Phi(:,end));
M_0 = dynMatrices(cd.Phi(:,1));
M_p = dynMatrices(delq*cd.Phi(:,end));

KE_after = (Delqd*cd.d_Phi(:,end))' * M_p * Delqd*cd.d_Phi(:,end) * td2m;
KE_before = cd.d_Phi(:,1)' * M_0 * cd.d_Phi(:,1) * thdsq_0;
DelKE_act = KE_after - KE_before;

c(1) = DelKE - DelKE_act;

if size(alpha_p,1) > 2
    for i = length(theta_p)-1 : -1 : 2
        p = endSwingFoot(bezConstraint(theta_p, alpha_p, theta_p(i)), ...
            [0,0]);
        ind = find(p(1) > sigma(:,1), 1);
        height = sigma(ind, 2);
        c(i) = height - p(2);
    end
else
    % CG only
    N = length(theta_p) - 1;
    s = (0 - theta_p(1)) / (theta_p(end) - theta_p(1));
    phi = 0;
    for i = 0 : N
        phi = phi + nchoosek(N,i) * (1-s)^(N-i) * s^i * alpha_p(:,i+1);
    end
    c(2) = -phi;
end
end

% x is (alpha(:,3:end-2))(:)
function [theta_p, alpha_p] = getCoefficients(x, th_ends, al_ends, deg)
theta_p = linspace(th_ends(1), th_ends(2), deg+1);
alpha_p_infix = reshape(x, length(x)/(deg-3), deg-3);
alpha_p = [al_ends(:,1:2), alpha_p_infix, al_ends(:,3:4)];
end

function cd = getOrMakeConstr(theta_p, alpha_p, grid_num)
persistent constr
if ~isempty(constr) 
    if constr.alpha_p == alpha_p
        cd = constr;
        return;
    end
end
cd = makeConstr(theta_p, alpha_p, grid_num);
constr = cd;
end
        