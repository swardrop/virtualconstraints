function [alpha, beta, gamma] = ZeroDynMSD(~, x)

% Undamped mass on a spring and damper without gravitational effects:
% ma + kv^2 + kx= 0
alpha = 1;
beta = 0.5;
gamma = 5*x;

end