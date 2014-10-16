function [nx, ny, nq, nk, xs, ys, qs, kes] = libParams

nx = 1;
ny = 3;
nq = 1;
nk = 11;

[~,l] = dynParams;
step_len_min = 0.3*l;
step_len_max = 0.5*l;
step_height_min = -0.1*l;
step_height_max = 0.1*l;

xs = linspace(step_len_min, step_len_max, nx);
ys = linspace(step_height_min, step_height_max, ny);

qs = 0; % Not used for CG model

% Determine the range in DelKE by choosing a nominal final configuration
% (flat ground) and adding a set velocity.
nom_qf = pi/6*[2,1];
max_Delqd = pi*[0.25;0.125];
M = dynMatrices(nom_qf);

max_DelKE = max_Delqd' * M * max_Delqd;
min_DelKE = -max_DelKE;

kes = linspace(min_DelKE, max_DelKE, nk);

end