function [t1, t2, h] = plotBez(points)

t2 = 0;
t = (0:0.01:1)';
n = size(points,1) - 1;

for i = 0 : n
    t2 = t2 + nchoosek(n,i) * (1-t).^(n-i) .* t.^i .* points(i+1,2);
end

t1 = t*(points(end,1) - points(1,1)) + points(1,1);

h = plot([0 4], [0 -8], 'b--', ... % Flat ground switching surface
    t1, t2, 'k', ... % Bezier curve
    points(:,1), points(:,2), 'rd', ...% Bezier control points
    points(:,1), points(:,2), 'r');

axis([pi/4 3/4*pi -3/2*pi -pi/2])
grid on

end