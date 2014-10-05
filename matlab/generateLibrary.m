function [lib, constrs] = generateLibrary(nx,ny,nq,nk,deg,optGrid,nomvel)
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

addpath model/active
addpath worker

if nargin < 7
    nomvel = 2; %rad/s
    if nargin < 6
        optGrid = 25;
    end
end
DelKE = DelKEs(nk);
% Produce arrray of all impact configurations and tree indexing this array
% by step length and height
[Q, Qtree] = impactConfigs(nx, ny, nq);
% Initialise constraints structure array
constrs(1:size(Q,2),1:nx,1:ny,1:nq,1:nk) = ...
    struct('theta_p', zeros(1,deg),  ...
          'alpha_p', zeros(1,deg), ...
          'Gamma_c', 0, 'Psi_c', -inf, ...
          'Gamma_f', 0, 'Psi_f', -inf, ...
          'Gamma_p', 0, 'Psi_p', -inf, ...
          'initq', 0, 'finalq', 0);
% Per element in Q, trace through Qtree and build a set of nk constraints
% per config at the leaves.
Qsize = size(Q,2);
lib(1:Qsize) = struct('initq', 0, 'step_len', Qtree);
for q = 1 : Qsize;
    initq = Q(:,q);
    lib(q).initq = initq;
    for l = 1 : nx          % For each step length in the tree:
        
        for h = 1 : ny          % For each step height for a given length:
            % Initialise array of primitives at leaf
            lib(q).step_len(l).step_ht(h).prims = zeros(1, nq*nk);
            
            for qf = 1 : nq         % For each configuration given l & h:
                finalq_ind = lib(q).step_len(l).step_ht(h).configs(qf);
                finalq = delq*Q(:,finalq_ind);
                sigma = makeGround(initq, finalq);
                for k = 1 : nk          % For every DelKE given a final q:
                    [vc, flag] = optimiseConstraint(initq, finalq, ...
                        DelKE(k), sigma, deg, optGrid);
                    if flag > 0 % Only add constraint if it is valid
                        vc.initq = q;
                        vc.finalq = finalq_ind;
                        constrs(q,l,h,qf,k) = vc;
                    end
                    lib(q).step_len(l).step_ht(h).prims(qf,k) = ...
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
            sortedind = sortConstrs(...
                constrs(lib(q).step_len(l).step_ht(h).prims),nomvel);
            lib(q).step_len(l).step_ht(h).prims = ...
                lib(q).step_len(l).step_ht(h).prims(sortedind);
        end
    end
end
end