% constrData = makeConstr(points)
%
% Takes a set of Bezier control points which are specified in each row of
% the points argument. Returns a structure containing the following fields:
% step_l - length of step
% step_h - height of step
% Gamma_c - value of Gamma function at critical theta
% Psi_c - value of Psi function at critical theta
% theta_c - critical theta
% Gamma_f - value of Gamma function just before impact
% Psi_f - value of Psi function just before impact
% points - Bezier control points (identical to supplied points)

function [constrData, Gamma, Psi, th_base] = makeConstr(points)

% Calculate step length and height
[~, ls] = getDynParams();
[s_l, s_h] = calcStepGeom(ls, points);

% Calculate partial solution
[Gamma, Psi, th_base, th_c] = PartialSolZeroDyn(points);

% Calculate Gamma and Psi at th_c.
step_size = th_base(2)-th_base(1);
idx_before = floor((th_c-th_base(1))/step_size) + 1; % 1-indexing
G_c = (Gamma(idx_before+1)-Gamma(idx_before))/step_size * ...
    (th_c - th_base(idx_before)) + Gamma(idx_before);
P_c = (Psi(idx_before+1)-Psi(idx_before))/step_size * ...
    (th_c - th_base(idx_before)) + Psi(idx_before);

constrData = struct;
constrData.step_l = s_l;
constrData.step_h = s_h;
constrData.Gamma_c = G_c;
constrData.Psi_c = P_c;
constrData.theta_c = th_c;
constrData.Gamma_f = Gamma(end);
constrData.Psi_f = Psi(end);
constrData.points = points;

end


function [length, height] = calcStepGeom(l, points)

x1 = getEndPoint(l, points(1,:), @cos);
x2 = getEndPoint(l, points(end,:), @cos);
length = x2 - x1;

y1 = getEndPoint(l, points(1,:), @sin);
y2 = getEndPoint(l, points(end,:), @sin);
height = y2 - y1;

end

function endpoint = getEndPoint(l, angs, trigfun)

endpoint = 0;
angle = 0;
for i = 1:length(l)
    angle = angle + angs(i);
    endpoint = endpoint + l(i)*trigfun(angle);
end

end