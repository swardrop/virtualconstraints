function [h, theta, q] = plotBez(theta_p, alpha_p)

s = 0:0.01:1;

theta = s*(theta_p(end) - theta_p(1)) + theta_p(1);
q = bezConstraint(theta_p, alpha_p, theta);

h = plot([-4 4], [-8 8], 'b--', ... % Flat ground switching surface
    theta, actuated(q), 'k', ... % Bezier curve
    theta_p, alpha_p, 'rd-'); % Bezier control points

min_w = pi/4;
min_h = pi/2;
minscale = 1.2;
ax_w = max(min_w);%, minscale*theta_p(end));
ax_h = max(min_h);%, minscale*max(max(alpha_p)));
axis([-ax_w ax_w -ax_h ax_h])
grid on
end