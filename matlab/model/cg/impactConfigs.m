function [Qtilde, tree] = impactConfigs(nx, ny, nq)
% Produces nx*ny*nq impact configurations for the robot. nx refers to the
% number of step lengths, ny to the number of step heights and nq to the
% number of individual configurations per couple (length, height).
% Returns the configurations as columns in Qtilde. tree is a structure
% which stores indexes into Qtilde on the basis of step length and height.

% For the compass-gait, nq must equal 1 - only 2DOF.
nq = 1;

% Limit step length and height based upon the robot's dimensions
[~,l] = dynParams;
step_len_min = 0.25*l;
step_len_max = 0.6*l;
step_height_min = -0.1*l;
step_height_max = 0.1*l;

step_len = linspace(step_len_min, step_len_max, nx);
step_height = linspace(step_height_min, step_height_max, ny);

% In general case, here we need to specify more so that the IK has only one
% solution per evaluation. This is not necessary for the CG.
% Something like <angle> = linspace(start, end, nq), perhaps.

% Build configuration tree
tree = struct;
for i = nx : -1 : 1
    tree.lens(i).len = step_len(i);
    for j = ny : -1 : 1
        tree.lens(i).hts(j).ht = step_height(j);
        for k = nq : -1 : 1 % Not necessary for CG
            ind = sub2ind([nx ny nq], i, j, k);
            Qtilde(ind,:) = solveIK(step_len(i), step_height(j));
            tree.lens(i).hts(j).cfgs(k) = ind;
        end
    end
end

% Sort by Qtilde by q1, then by q2, then by ... qn.
[Qtilde, ind_sorted] = sortrows(Qtilde);
Qtilde = Qtilde';
% Update indexes in tree
for i = 1:nx
    for j = 1:ny
        tree.lens(i).hts(j).cfgs = ind_sorted(tree.lens(i).hts(j).cfgs);
    end
end

end

% This function will have to change a bit for robots other than CG
function q = solveIK(step_len, step_ht)
q = [pi/4;0];
iter = 0;
gridsize = 21;
accept = false;

while ~accept
    q1m = q(1) - (pi/4)/(gridsize^iter);
    q1p = q(1) + (pi/4)/(gridsize^iter);
    q2m = q(2) - (pi/2)/(gridsize^iter);
    q2p = q(2) + (pi/2)/(gridsize^iter);
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