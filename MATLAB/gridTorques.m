function [alpha2, alpha3, cost] = ...
    gridTorques(th_end, al_end, N, thd_sq_0, type)
%gridTorques Takes a final control point of a Bezier curve constraint
% and produces a NxN grid of the p-norm of the torque resulting from the
% gridded alpha2 and alpha3 values.

% Set up constraint values
al_p(:,6) = al_end;
al_p(:,5) = al_end;
th_p(6) = th_end;
[b0, b_th0, b1] = invarianceCond(th_p, al_p, th_end, 6, true);
th_p(1) = b_th0;
al_p(:,1) = b0;
al_p(:,2) = b1;

cen2 = 2/5*(al_end-b0)+b0;
min2 = cen2 - pi/1.5;
max2 = cen2 + pi/1.5;
cen3 = 3/5*(al_end-b0)+b0;
min3 = cen3 - pi/1.5;
max3 = cen3 + pi/1.5;

alpha2 = linspace(min2, max2, N);
alpha2 = repmat(alpha2, N, 1);
alpha3 = linspace(min3, max3, N)';
alpha3 = repmat(alpha3, 1, N);

for i = 1 : size(alpha2, 1)
    for j = 1 : size(alpha2, 2)
        al_p(:,3) = alpha2(i,j);
        al_p(:,4) = alpha3(i,j);
        cd = makeConstr(th_p, al_p);
        u = nomTorque(cd, thd_sq_0);
        switch type
            case {'Integral', '1-norm'}
                cost(i,j) = sum(abs(u));
            case '2-norm'
                cost(i,j) = norm(u);
            case {'Max', 'Inf-norm'}
                cost(i,j) = max(u);
        end
        constrData(i,j).Gamma_c = cd.Gamma_c;
        constrData(i,j).Psi_c = cd.Psi_c;
    end
end

% Cut out any row or column which will blow out the scale
% inrange = cost < 3*median(cost(:));
% rowsInRange = sum(inrange,2);
% colsInRange = sum(inrange,1);
% alpha2 = removeOutOfRange(alpha2, rowsInRange, colsInRange, N);
% alpha3 = removeOutOfRange(alpha3, rowsInRange, colsInRange, N);
% cost = removeOutOfRange(cost, rowsInRange, colsInRange, N);

figure;
surfc(alpha2,alpha3,cost);
title([type ' of torque with a degree-5 Bezier polynomial constraint']);
xlabel('\alpha_2 (rad)');
ylabel('\alpha_3 (rad)');
zlabel('Cost');

orderings(constrData, thd_sq_0);

end

function mat = removeOutOfRange(mat, rowsInRange, colsInRange, N)
mat(rowsInRange<N/2, :) = [];
mat(:, colsInRange<N/2) = [];
end