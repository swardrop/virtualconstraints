function [alpha, beta, gamma] = ZeroDynMassSpring(~, x)

% Undamped mass on a spring without gravitational effects:
% ma + kx = 0
alpha = 1;
beta = 0;
gamma = 0.5*x;

end