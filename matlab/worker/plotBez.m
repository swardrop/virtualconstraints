function [h, theta, q] = plotBez(theta_p, alpha_p, th_grid)

if nargin < 3
    th_grid = 20;
end
s = linspace(0, 1, th_grid);

theta = s*(theta_p(end) - theta_p(1)) + theta_p(1);
q = bezConstraint(theta_p, alpha_p, theta);

h = plot([-4 4], [-8 8], 'b--', ... % Flat ground switching surface
    theta, actuated(q), 'k', ... % Bezier curve
    theta_p, alpha_p, 'rd-'); % Bezier control points

min_w = pi/4;
min_h = pi/2;
minscale = 1.2;
ax_w = max(min_w, minscale*theta_p(end));
ax_h = max(min_h, minscale*max(max(alpha_p)));
axis([-ax_w ax_w -ax_h ax_h])
xlabel('\theta')
ylabel('H_0 q')
grid on
end