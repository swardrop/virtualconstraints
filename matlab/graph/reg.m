cd ..
q0 = -pi/12*[2;1];
qf = -q0;
grid = 50;
DelKE = [-0.1 0 0.1];
deg = 7;
regs = [1 10 100 500 1000 1e4 1e5 1e6];

cost = zeros(length(regs), length(DelKE));
avs = zeros(size(regs));
for r = 1 : length(regs)
    t = zeros(1,3);
    clear optimiseConstraint
    for k = 1:length(DelKE)
        tic;
        vc = optimiseConstraint(q0,qf,DelKE(k),[],deg,grid, regs(r));
        t(k) = toc;
        vc = makeConstr(vc.theta_p, vc.alpha_p,2000);
        thdsq = thdsq_nom(vc);
        cost(r,k) = norm(nomTorque(vc,thdsq));
    end
    avs(r) = mean(t);
end

cd graph