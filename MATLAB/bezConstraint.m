function q = bezConstraint(theta_p, alpha_p, theta)
% Produces the nominal value of the state evaluated at theta given the
% constraint defined by theta_p and alpha_p

s = (theta - theta_p(1)) / (theta_p(end) - theta_p(1));
s(s>1) = 1;
s(s<0) = 0;
s = repmat(s,size(alpha_p,1),1);
q_dep = zeros([size(alpha_p, 1), 1]);
n = length(theta_p) - 1;
for i = 0 : n
    q_dep = q_dep + nchoosek(n,i).*(1-s).^(n-i) .* s.^i .* alpha_p(:,i+1);
end

H = constrMatrices;
% q = zeros(size(alpha_p,1)+1, length(theta));
% for jj = length(theta):-1:1
%     q(:,jj) = H*[q_dep(:,jj); theta(:,jj)];
% end
q = H*[q_dep;theta];