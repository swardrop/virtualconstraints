function [Qtilde, tree] = impactConfigs
% Produces nx*ny*nq impact configurations for the robot. nx refers to the
% number of step lengths, ny to the number of step heights and nq to the
% number of individual configurations per couple (length, height).
% Returns the configurations as columns in Qtilde. tree is a structure
% which stores indexes into Qtilde on the basis of step length and height.

[nx, ny, nq, ~, step_len, step_height, qs] = libParams;

% In general case, here we need to specify more so that the IK has only one
% solution per evaluation. This is not necessary for the CG.
% Something like <angle> = linspace(start, end, nq), perhaps.

% Build configuration tree
tree(nx) = struct;
Qtilde = zeros(2,nx,ny,nq);

parfor i = 1:nx
    tree(i).len = step_len(i);
    % Preallocate
    tree(i).step_ht(ny) = struct;
    for j = 1:ny
        tree(i).step_ht(j).ht = step_height(j);
        tree(i).step_ht(j).configs(nq) = 0;
        for q = 1:nq % Not necessary for CG
            ind = sub2ind([nx ny nq], i, j, q);
            Qtilde(:,i,j,q) = delq*solveIK(step_len(i), ...
                step_height(j), qs(q));
            tree(i).step_ht(j).configs(q) = ind;
        end
    end
end

% Sort by Qtilde by q1, then by q2, then by ... qn.
Qtilde = reshape(Qtilde, length(delq), nx*ny*nq);
[Qtilde, ind_sorted] = sortrows(Qtilde');
[~,ind_sorted] = sort(ind_sorted);
Qtilde = Qtilde';
% Update indexes in tree
parfor i = 1:nx;
    for j = 1:ny
        tree(i).step_ht(j).configs = ...
            ind_sorted(tree(i).step_ht(j).configs);
    end
end

end