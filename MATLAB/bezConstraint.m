function q = bezConstraint(theta_p, alpha_p, theta)
% Produces the nominal value of the state evaluated at theta given the
% constraint defined by theta_p and alpha_p

s = (theta - theta_p(1)) / (theta_p(end) - theta_p(1));
s(s>1) = 1;
s(s<0) = 0;
q_dep = zeros([size(alpha_p, 1), length(theta)]);
n = length(theta_p) - 1;

for i = 1 : size(alpha_p,1)
    for k = 0 : n
        q_dep(i,:) = q_dep(i,:) + nchoosek(n,k)*(1-s).^(n-k) ...
            .* s.^k * alpha_p(i,k+1);
    end
end

H = constrMatrices;
q = zeros(size(alpha_p,1)+1, length(theta));
for jj = length(theta):-1:1
    q(:,jj) = H*[q_dep(:,jj); theta(:,jj)];
end
q = H*[q_dep;theta];