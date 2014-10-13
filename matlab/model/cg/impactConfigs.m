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
Qtilde = Qtilde';
% Update indexes in tree
parfor i = 1:nx;
    for j = 1:ny
        tree(i).step_ht(j).configs = ...
            ind_sorted(tree(i).step_ht(j).configs);
    end
end

end

% This function will have to change a bit for robots other than CG
function q = solveIK(step_len, step_ht, q_rest)
q = [pi/4;0];
iter = 0;
gridsize = 20;
accept = false;

while ~accept
    q1m = q(1) - (pi/4)/((gridsize/2)^iter);
    q1p = q(1) + (pi/4)/((gridsize/2)^iter);
    q2m = q(2) - (pi/2)/((gridsize/2)^iter);
    q2p = q(2) + (pi/2)/((gridsize/2)^iter);
    iter = iter + 1;
    [q, accept] = mingrid(step_len, step_ht, q1m, q1p, q2m, q2p, gridsize);
end
end

function [qmin, accept] = mingrid(len, ht, q1m, q1p, q2m, q2p, gridsize)
tol = 1e-3;
[~,l] = dynParams;
% Create fairly sparse grid of q1 and q2 values
[q1, q2] = meshgrid(linspace(q1m,q1p,gridsize),linspace(q2m,q2p,gridsize));

x = l*sin(q2) + l*sin(q1-q2);
y = l*cos(q2) - l*cos(q1-q2);

dx = len - x;
dy = ht - y;
dist = sqrt(dx.^2 + dy.^2);

[colmin, rowidx] = min(dist);
[totmin, colidx] = min(colmin);

qmin(1,1) = q1(rowidx(colidx), colidx);
qmin(2,1) = q2(rowidx(colidx), colidx);

if totmin < tol
    accept = true;
else
    accept = false;
end
end