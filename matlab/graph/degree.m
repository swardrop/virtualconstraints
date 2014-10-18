cd ..
degs = [4 5 7 10 15 25];
q0 = -pi/12*[2;1];
qf = -q0;
DelKE = [-0.1 0 0.1];
grid = 50;

avs = zeros(size(degs));
cost = zeros(length(degs),length(DelKE));

for d = 1:length(degs)
    t = zeros(1,3);
    clear optimiseConstraint
    for k = 1:length(DelKE)
        tic;
        vc = optimiseConstraint(q0,qf,DelKE(k),[],degs(d),grid);
        t(k) = toc;
        vc = makeConstr(vc.theta_p, vc.alpha_p,2000);
        thdsq = thdsq_nom(vc);
        cost(d,k) = norm(nomTorque(vc,thdsq));
    end
    avs(d) = mean(t);
end
cd graph