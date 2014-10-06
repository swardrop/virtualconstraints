cd ..
theta_p = [-0.2618 -0.1745 -0.0873 0 0.0873 0.1745 0.2618];
alpha_p = [-0.5236 -0.4803 -0.5322 0 4.6398 0.5236 0.5236];

[Gamma, Psi, th_base, th_c, alpha] = ...
    PartialSolZeroDyn(theta_p, alpha_p, 1000);

subplot(2,1,1)
plot(th_base, Gamma, 'k');
ylabel('\Gamma(\theta)');
xlabel('\theta');
xlim([-0.3 0.3]);

subplot(2,1,2)
plot(th_base, Psi, 'k');
ylabel('\Psi(\theta) (rad/s)^2');
xlabel('\theta');
xlim([-0.3 0.3]);

figure
plot(th_base, alpha, 'k');
ylabel('\alpha(\theta)')
xlabel('\theta');
xlim([-0.3 0.3]);

figure
[~,~,q] = plotBez(theta_p, alpha_p,1000);
clf
visualiseStatic(q)
xlim([-0.6,1.2]);
grid off
xlabel('Horizontal displacement from stance foot (m)');
ylabel('Vertical displacement from stance foot (m)');
title('Cartesian robot path')