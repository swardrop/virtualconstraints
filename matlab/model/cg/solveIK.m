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