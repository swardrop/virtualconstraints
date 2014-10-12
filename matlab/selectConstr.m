function [constr, success] = selectConstr(L, P, q, qd, sigma, x, k)
% Selects a motion primitive on the basis of the terrain ahead, with a
% k-step lookahead for feasibility analysis

for i = size(L(1).step_len) : -1 : 1
    lens(i) = L(q).step_len(i).len;
end
q_ind = impactCondSearch(L, q);
b = 5; % Max phase vel (rad/s)
[constr, success] = addNode(L,P,q_ind,qd,sigma,lens,x,b,k);
end

function [constr, success] = addNode(L, P, q_ind, qd, sigma, x, lens, b, k)

[~,~,c] = constrMatrices;
v0 = (c*qd)^2;
for i = length(lens) : -1 : 1
    j = quantise(height(sigma, x, lens(i)));
    [p_idx(i), p_peers] = treeSearch(L, q_ind, i, j, v0);
    p(i) = P(p_idx);
end

while true
    for i = length(p) : -1 : 1 
        vf = p.Gamma_f*v0 + p.Psi_f;
        if vf <= b^2 && collisionFree(p, sigma, x)
            vc(i) = p.Gamma_c*v0 + p.Psi;
        else
            p_idx(i) = p_idx(i)+1;
            if p_idx(i) <= length(p_peers)
                p(i) = p_peers(i-1);
            else
                p(i).finalq = 0; % Mark primitive as having no successor
            end
            vc(i) = Inf;
        end
    end
    v = sort(vc);
    idx = 1;
    % Up to L27. Need to consider whether to loop here or use while loop
end

end





function [p_idx, peers] = treeSearch(L, q_ind, i, j, v0)
peers = L(q_ind).step_len(i).step_ht(j).prims;
p_idx = binarySearch(v0-L(q_ind).step_len(i).step_ht(j).sortkeys, 0);
end


function q_ind = impactCondSearch(L, q)

end

function isColFree = collisionFree(p, sigma, x)

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

function idx = binarySearch(array, val)
% Returns the highest index for which array(ind) >= val. Assumes that the
% array is ordered in DESCENDING order

min = 1; max = length(array); % Inclusive lb and ub
while (min < max)
    idx = floor((max+min)/2);
    if (array(idx) > val)
        min = idx+1;
    else
        max = idx;
    end
end
end