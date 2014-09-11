function [alpha2, alpha3, torque] = gridTorques(final_point, N, thd_0_sq)
%gridTorques Takes a final control point of a Bezier curve constraint
% and produces a NxN grid of the 2-norm of the torque resuling from the
% gridded alpha2 and alpha3 values.

% Set up constraint values
min2 = -3*pi/2;
max2 = -pi/2;
min3 = -3*pi/2;
max3 = -2.5;

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
        u = nomTorqueCG(constr, thd_0_sq);
        torque(i,j) = norm(u);
    end
end

surf(alpha2,alpha3,torque);
title('2-Norm of torque with a degree-5 polynomial Bezier constraint');
xlabel('\alpha_2 (rad)');
ylabel('\alpha_3 (rad)');
zlabel('Torque (Nm)');