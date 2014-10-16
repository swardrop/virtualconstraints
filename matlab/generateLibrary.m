function [L, P] = generateLibrary(name, deg,optGrid,minvel)
%generateLibrary Produces a complete libary of motion primitives for the
% robot whose dynamics are expressed in the functions dynMatrices and
% impactMatries and which have control applied as defined in
% constrMatrices, where H0*q are the actuated coordinates and c*q is the
% unactuated coordinate.
%   lib is a structure array with the following fields:
%   - init_config   - the initial configuration of the vcs
%   - step_len      - an array, sorted  by step length, of arrays:
%                       - Each array is sorted by step_height, containing
%                       a sorted array indexes to virtual constraints in
%                       the constrs array
%   constrs is a structure array containing all virtual constraints. The
%   virtual constraints contain indexes into lib based on their initial and
%   final configurations.
%
% NOTE in order to make use of parallel execution, use
%   matlabpool open <numthreads>
% before calling generateLibrary. Close the threads with
%   matlabpool close

if nargin < 2
    deg = 10;
    if nargin < 3
        minvel = 0.5; %rad/s
        if nargin < 4
            optGrid = 25;
        end
    end
end
clear optimiseConstraint
[nx, ny, nq, nk, ~, ~, ~, kes] = libParams;
% Produce arrray of all impact configurations and tree indexing this array
% by step length and height
disp('Producing impact configurations...');
[Q, Qtree] = impactConfigs;
% Initialise constraints structure array
P(1:size(Q,2),1:nx,1:ny,1:nq,1:nk) = ...
    struct('theta_p', zeros(1,deg),  ...
          'alpha_p', zeros(1,deg), ...
          'Gamma_c', 0, 'Psi_c', -inf, ...
          'Gamma_f', 0, 'Psi_f', -inf, ...
          'Gamma_p', 0, 'Psi_p', -inf, ...
          'initq', 0, 'finalq', 0);
% Per element in Q, trace through Qtree and build a set of nk constraints
% per config at the leaves.
Qsize = size(Q,2);
L(1:Qsize) = struct('initq', 0, 'step_len', Qtree);
parfor q = 1 : Qsize;
    fprintf('Generating constraints for impact config %d of %d...\n', q, Qsize);
    initq = Q(:,q);
    L(q).initq = initq;
    for l = 1 : nx          % For each step length in the tree:
        
        for h = 1 : ny          % For each step height for a given length:
            % Initialise array of primitives at leaf
            L(q).step_len(l).step_ht(h).prims = zeros(1, nq*nk);
            
            for qf = 1 : nq         % For each configuration given l & h:
                finalq_ind = L(q).step_len(l).step_ht(h).configs(qf);
                finalq = delq*Q(:,finalq_ind);
                sigma = makeGround(initq, finalq);
                for k = 1 : nk          % For every DelKE given a final q:
                    fprintf('\t%d> Optimising constraint %d of %d...', q,...
                        k + nk*(qf-1 + nq*(h-1 + ny*(l-1))), nx*ny*nq*nk);
                    try
                        [vc, flag] = optimiseConstraint(initq, finalq, ...
                            kes(k), sigma, deg, optGrid);
                        if flag > 0 % Only add constraint if it is valid
                            fprintf('Success\n');
                            vc.initq = q;
                            vc.finalq = finalq_ind;
                            P(q,l,h,qf,k) = vc;
                        else
                            fprintf('Failure\n');
                        end
                    catch
                        fprintf('Hard Failure!\n');
                    end
                    L(q).step_len(l).step_ht(h).prims(qf,k) = ...
                        sub2ind([Qsize,nx,ny,nq,nk], q,l,h,qf,k);
                end
            end
        end
    end
end

% Sort the list of nq*nk constraints corresponding to each initq, l & h.
parfor q = 1 : Qsize;
    for l = 1 : nx
        for h = 1 : ny
            [sortkeys, sortedind] = sortConstrs(...
                P(L(q).step_len(l).step_ht(h).prims),minvel);
            [~,sortedind] = sort(sortedind);
            L(q).step_len(l).step_ht(h).prims = ...
                L(q).step_len(l).step_ht(h).prims(sortedind);
            L(q).step_len(l).step_ht(h).sortkeys = sortkeys;
        end
    end
end
% Save library to MAT file
if nargin < 1
    name = 'lib.mat';
end
name = ['lib/', name];
save(name, 'L', 'P');
end