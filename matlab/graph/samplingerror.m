function [times, nom_vel, post_vel] = samplingerror(theta_p, alpha_p)
cd ..

samples = [10, 25, 50, 200];
times = zeros(size(samples));
nom_vel = zeros(size(samples));
post_vel = zeros(size(samples));
color = [0.5, 0.5, 0.5];
style = {'-.', '--', ':', '-'};
figure('Position',[300,300,1200,400])
for i = 1 : length(samples)
    start = [alpha_p(:,1); theta_p(1)]; % pre multiply by H^-1 in general
    fin = [alpha_p(:,end); theta_p(end)];
    tic
    sp = optimiseConstraint(start, fin, 0, [], 6, samples(i));
    times(i) = toc;
    subplot(1,2,1)
    plot(sp.th_base, sp.Gamma, style{i}, 'Color', color);
    hold on
    subplot(1,2,2)
    plot(sp.th_base, sp.Psi, style{i}, 'Color', color);
    hold on
    color = color - 0.1;
    
    [~, Phi_f, dPhi_0, dPhi_f] = constrEndPts(theta_p, alpha_p);
    nom_vel(i) = thdsq_nom(sp, 0);
    td2m = sp.Gamma_f*nom_vel(i) + sp.Psi_f;
    Delqd = impactMatrices(Phi_f);
    post_vel(i) = [0 1]*Delqd*dPhi_f*sqrt(td2m);
end

subplot(1,2,1)
grid on
xlabel('\theta')
ylabel('\Gamma(\theta)')
h = legend('10', '25', '50', '200', 'Location', 'SouthEast');
h = get(h,'title');
set(h,'String','Samples','FontWeight','bold');
subplot(1,2,2)
grid on
xlabel('\theta')
ylabel('\Psi(\theta)')
h = legend('10', '25', '50', '200', 'Location', 'SouthEast');
h = get(h,'title');
set(h,'String','Samples','FontWeight','bold');
end