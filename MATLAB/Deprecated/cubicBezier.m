% This function plots the cubic Bezier curve defined by the four points P0,
% P1, P2 and P3.
function [th1, th2, h] = cubicBezier(P0, P1, P2, P3)

t = 0:0.01:1;
B = P0*((1 - t).^3) + 3*P1*(((1 - t).^2).*t) + ...
    3*P2*((1 - t).*t.^2) + P3*(t.^3);

th1 = B(1, :);
th2 = B(2, :);

controlPoints = [P0 P1 P2 P3];


h = plot([P0(1) P3(1)], [P0(2) P3(2)], 'b--', ... % Straight line
th1, th2, 'k', ... % Bezier curve
controlPoints(1,:), controlPoints(2,:), 'rd', ...% Bezier control points
controlPoints(1,1:2), controlPoints(2,1:2), 'r', ...
controlPoints(1,3:4), controlPoints(2,3:4), 'r', ...
(P0(1)+P3(1))/2, (P0(2)+P3(2))/2, 'kx');  % Midpoint of line

axis([0 pi -2*pi 0])

end