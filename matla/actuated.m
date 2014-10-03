function q_act = actuated(q)
% Actuated coordinates in terms of the generalised coordinates q
[~,H0] = constrMatrices;
q_act = zeros(size(H0,1),size(q,2));
for jj = size(q,2):-1:1
    q_act(:,jj) = H0*q(:,jj);
end
end