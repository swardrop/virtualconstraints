function [alpha2, alpha3, cost] = ...
    gridTorques(final_point, N, thd_0_sq, type)
%gridTorques Takes a final control point of a Bezier curve constraint
% and produces a NxN grid of the 2-norm of the torque resuling from the
% gridded alpha2 and alpha3 values.

% Set up constraint values
min2 = -3*pi/2;
max2 = -pi/2;
min3 = -3*pi/2;
max3 = -pi/2;

alpha2 = linspace(min2, max2, N);
alpha2 = repmat(alpha2, N, 1);
alpha3 = linspace(min3, max3, N)';
alpha3 = repmat(alpha3, 1, N);

constr(5,:) = final_point;
[b0, b_th0, b1] = invarianceCond(constr, final_point(1), 5, true);
constr(1,1) = b_th0;
constr(1,2) = b0;
constr(2,2) = b1;

for i = 2 : 4
    constr(i,1) = (i-1)*(constr(5,1)-constr(1,1))/(5-1) + constr(1,1);
end

for i = 1 : size(alpha2, 1)
    for j = 1 : size(alpha2, 2)
        constr(3,2) = alpha2(i,j);
        constr(4,2) = alpha3(i,j);
        [u,~,G_c,P_c] = nomTorqueCG(constr, thd_0_sq);
        switch type
            case {'Integral', '1-norm'}
                cost(i,j) = sum(abs(u));
            case '2-norm'
                cost(i,j) = norm(u);
            case {'Max', 'Inf-norm'}
                cost(i,j) = max(u);
        end
        constrData(i,j).Gamma_c = G_c;
        constrData(i,j).Psi_c = P_c;
    end
end

% Cut out any row or column which will blow out the scale
inrange = cost < 3*median(cost(:));
rowsInRange = sum(inrange,2);
colsInRange = sum(inrange,1);
alpha2 = removeOutOfRange(alpha2, rowsInRange, colsInRange, N);
alpha3 = removeOutOfRange(alpha3, rowsInRange, colsInRange, N);
cost = removeOutOfRange(cost, rowsInRange, colsInRange, N);

figure;
surfc(alpha2,alpha3,cost);
title([type ' of torque with a degree-5 Bezier polynomial constraint']);
xlabel('\alpha_2 (rad)');
ylabel('\alpha_3 (rad)');
zlabel('Cost');

orderings(constrData, thd_0_sq);

end

function mat = removeOutOfRange(mat, rowsInRange, colsInRange, N)
mat(rowsInRange<N/2, :) = [];
mat(:, colsInRange<N/2) = [];
end