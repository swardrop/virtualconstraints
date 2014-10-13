function [constr, success] = selectConstr(L, P, q, qd, sigma, x, k)
% Selects a motion primitive on the basis of the terrain ahead, with a
% k-step lookahead for feasibility analysis

q_ind = impactCondSearch(L, q);
for i = size(L(1).step_len,2) : -1 : 1
    lens(i) = L(q_ind).step_len(i).len;
end
b = inf; % Max phase vel (rad/s)
[constr, success] = addNode(L,P,q_ind,qd,sigma,lens,x,b,k);
end

function [constr, success] = addNode(L, P, q_ind, qd, sigma, lens, x, b, k)

[~,~,c] = constrMatrices;
v0 = (c*qd)^2;
for i = length(lens) : -1 : 1
    j = quantise(height(sigma, x, lens(i)));
    [p_idx(i), p_peers{i}, searchSucc(i)] = treeSearch(L, q_ind, i, j, v0);
    p(i) = P(p_peers{i}(p_idx(i)));
    if ~searchSucc(i)
        [p(i), p_idx(i)] = nextBest(P, p_peers{i}, p_idx(i));
    end
end
if ~any(searchSucc)
    success = false;
    constr = [];
    return
end
while true
    for i = length(p) : -1 : 1 
        vf = p(i).Gamma_f*v0 + p(i).Psi_f;
        if vf <= b^2 && collisionFree(p(i), sigma, x)
            vc(i) = p(i).Gamma_c*v0 + p(i).Psi_c;
        else
            [p(i), p_idx(i)] = nextBest(P, p_peers{i}, p_idx(i));
            vc(i) = Inf;
        end
    end
    vc = sort(vc);
    for i = length(p)
        if vc(i) == Inf
            break;
        end
        if k == 1
            constr = p(i);
            success = true;
            return
        end
        qp_ind = p(i).finalq;
        [~,~,Phi_f,dPhi_f] = constrEndPts(p(i).theta_p, p(i).alpha_p);
        qdp = dPhi_f*sqrt(p(i).Gamma_f*v0 + p(i).Psi_f);
        p2 = endSwingFoot(Phi_f, [x, 0]);
        xp = x + p2(1);
        [~, success] = addNode(L, P, qp_ind, qdp, sigma, lens, xp, b, k-1);
        if success
            constr = p(i);
            return
        else
            [p(i), p_idx(i)] = nextBest(P, p_peers{i}, p_idx(i));
        end
    end
    test = [p.finalq];
    if ~any(test)
        constr = [];
        success = false;
        return
    end
end

end





function [p_idx, peers, success] = treeSearch(L, q_ind, i, j, v0)
peers = L(q_ind).step_len(i).step_ht(j).prims;
[p_idx, success] = ...
    binarySearch(v0-L(q_ind).step_len(i).step_ht(j).sortkeys, 0);
end


function q_ind = impactCondSearch(L, q)
qs = [L.initq];
% Search first for the first coordinate in q, then if the search hasn't yet
% resolved, search over all of the duplicate values of q1 for q2 values,
% etc, until there is only one configuration.

q_ind = binarySearch(q(1) - qs(1,:), 0);
dim = 1;
while q_ind < size(qs,2) && (qs(dim,q_ind) == qs(dim,q_ind+1))
    consec_array = qs(:,q_ind:q_ind+1);
    consec_ind = q_ind+2;
    while consec_ind <= size(qs,2) && qs(dim, consec_ind) == q(dim)
        consec_array(:,end+1) = qs(dim, consec_ind); %#ok<AGROW>
        consec_ind = consec_ind + 1;
    end
    qs = consec_array;
    dim = dim + 1;
    q_ind = binarySearch(q(dim) - consec_array(dim,:), 0) + q_ind - 1;
end
end

function isColFree = collisionFree(p, sigma, x)
isColFree = true; % Need to replace this with proper check
end

function [p, p_idx] = nextBest(P, p_peers, p_idx)
p_idx = p_idx-1;
if p_idx > 0
    p = P(p_peers(p_idx));
else
    p = P(p_peers(p_idx+1));
    p.finalq = 0; % Mark primitive as having no successor
end
end

function h_ind = quantise(h)
[~, ~, ~, ~, ~, ys] = libParams;
h_ind = binarySearch(h-ys, 0);
h_ind = h_ind(1);
end

function h = height(sigma, x, len)
ind = x + len > sigma(:,1);
h = sigma(ind(1), 2);
end

function [idx, success] = binarySearch(array, val)
% Returns the highest index for which array(ind) >= val. Assumes that the
% array is ordered in DESCENDING order

min = 1; max = length(array); % Inclusive lb and ub
while (min < max-1)
    idx = floor((max+min)/2);
    if (array(idx) < val)
        max = idx-1;
    else
        min = idx;
    end
end
success = true;
if array(max) >= val
    idx = max;
elseif array(min) >= val
    idx = min;
else
    success = false;
end
end