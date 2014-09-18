function theta = phasevar(q)
[~,~,c] = constrMatrices;
theta = zeros(size(q,2));
for jj = length(theta):-1:1
    theta(jj) = c*q(:,jj);
end
end