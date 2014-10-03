function [times, nom_vel_s, nom_vel, post_vel, KE_bf, DelKE] = ...
    samplingerror(theta_p, alpha_p)
cd ..

samples = [5, 10, 25, 50, 200];
times = zeros(size(samples));
nom_vel_s = zeros(size(samples));
nom_vel = zeros(size(samples));
post_vel = zeros(size(samples));
KE_bf = zeros(size(samples));
DelKE = zeros(size(samples));
color = [0.8, 0.8, 0.8];
style = {'--', '-', '-.', ':', '--'};
figure('Position',[300,300,1200,400])

[~, Phi_f, dPhi_0, dPhi_f] = constrEndPts(theta_p, alpha_p);
Delqd = impactMatrices(Phi_f);
M = dynMatrices(delq*Phi_f, dPhi_f);

for i = 1 : length(samples)
    % Visualise error in identical constraint coefficients
    sc = makeConstr(theta_p, alpha_p, samples(i));
    subplot(1,2,1)
    plot(sc.th_base, sc.Gamma, style{i}, 'Color', color);
    hold on
    subplot(1,2,2)
    plot(sc.th_base, sc.Psi, style{i}, 'Color', color);
    hold on
    color = color - 0.8/5;
    
    % Calculate error from optimisation
    start = [alpha_p(:,1); theta_p(1)]; % pre multiply by H^-1 in general
    fin = [alpha_p(:,end); theta_p(end)];
    tic
    oc = optimiseConstraint(start, fin, 0, [], 6, samples(i));
    times(i) = toc;
    oc_act = makeConstr(oc.theta_p, oc.alpha_p, 2000);
    nom_vel_s(i) = thdsq_nom(oc, 0);
    nom_vel(i) = thdsq_nom(oc_act, 0);
    td2m = oc_act.Gamma_f*nom_vel(i) + oc_act.Psi_f;
    post_vel(i) = [0 1]*Delqd*dPhi_f*sqrt(td2m);

    KE_af = (Delqd*dPhi_f)' * M * Delqd*dPhi_f * td2m;
    KE_bf(i) = dPhi_0' * M * dPhi_0 * nom_vel(i);
    DelKE(i) = KE_af - KE_bf(i);
end

sc = makeConstr(theta_p, alpha_p, 2000);
subplot(1,2,1)
plot(sc.th_base, sc.Gamma, 'k-');
grid on
xlabel('\theta')
ylabel('\Gamma(\theta)')
h = legend('5', '10', '25', '50', '200', '2000', 'Location', 'Best');
h = get(h,'title');
set(h,'String','Samples','FontWeight','bold');
xlim([-0.3 0.3]);
subplot(1,2,2)
plot(sc.th_base, sc.Psi, 'k-');
grid on
xlabel('\theta')
ylabel('\Psi(\theta)')
h = legend('5', '10', '25', '50', '200', '2000', 'Location', 'Best');
h = get(h,'title');
set(h,'String','Samples','FontWeight','bold');
xlim([-0.3 0.3]);
end